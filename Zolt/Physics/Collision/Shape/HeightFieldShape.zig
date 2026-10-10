//! Port of: Jolt/Physics/Collision/Shape/HeightFieldShape.h, Jolt/Physics/Collision/Shape/HeightFieldShape.cpp
//! Status: complete
//! Not ported: JPH_DEBUG_HEIGHT_FIELD (commented out developer switch that draws the bounding boxes and triangles
//!   while walking the height field)
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2): `HeightFieldShape` derives from
//! `Shape` directly (`base: Shape`), `overrides` lists every C++ `override` in header order.
//! - The namespace `HeightFieldShapeConstants` is a struct of constants (`cNoCollisionValue` -> `no_collision_value`).
//! - Settings: `HeightFieldShapeSettings` owns its arrays (`height_samples`, `material_indices`, `materials`), allocated
//!   with the settings' allocator (`base.allocator`) and freed by the destructor chain. The C++ constructor
//!   `HeightFieldShapeSettings(inSamples, inOffset, inScale, inSampleCount, inMaterialIndices, inMaterialList)` is
//!   `init(allocator, samples, offset, scale, sample_count, .{ .material_indices, .materials })`; it copies the arrays,
//!   so it returns `Allocator.Error`. `DetermineMinAndMaxSample` returns `MinMaxSample` (out parameters).
//! - The shape owns one buffer like Jolt's single AlignedAllocate block: the range blocks, the height samples and the
//!   active edges are slices into it (`range_blocks`, `height_samples`, `active_edges`), allocated with the shape's
//!   allocator. The temporary arrays of the constructor use the allocator passed to `initFromSettings`, the active edge
//!   calculation of the constructor runs on a `TempAllocatorMalloc` over that allocator (Jolt uses a
//!   TempAllocatorMalloc over its global allocator).
//! - The mutating functions `setHeights` / `setMaterials` take `*HeightFieldShape` (Rule M) and the caller's
//!   `TempAllocator`; they return `Allocator.Error` where Jolt aborts on out of memory. `setMaterials` grows the
//!   material list with the shape's allocator. When it runs out of memory the shape stays valid, the material list may
//!   have grown by the materials that were added before (they are unused).
//! - Raw pointer + stride parameters stay raw: `getHeights(..., out_heights: [*]f32, heights_stride: isize)`,
//!   `setHeights(..., heights: [*]const f32, heights_stride: isize, ...)`, the same for `getMaterials` /
//!   `setMaterials` (a stride can be negative, the data is then upside down). The material list of `setMaterials` is
//!   `?[]const PhysicsMaterialRefC` (the items of a PhysicsMaterialList, null: keep the current list).
//! - Out parameters become structs: `ProjectOntoSurface` returns `?SurfacePosition`, `GetSubShapeCoordinates`
//!   returns `SubShapeCoordinates`, the private `GetPosition(x, y, offset, scale, outNoCollision)` overload is
//!   `getPositionInBlock` (returns `PositionInBlock`), `GetBlockOffsetAndScale` / `sGetRangeBlockOffsetAndStride` /
//!   `GetRangeBlock` / `DecodeSubShapeID` return structs.
//! - Overloads: `GetMaterial(inX, inY)` is `getMaterialAt` (the virtual `getMaterial(sub_shape_id)` has the plain
//!   name), `CalculateActiveEdges(const HeightFieldShapeSettings &)` is `calculateActiveEdgesFromSettings`.
//! - `WalkHeightField` / `DecodingContext::WalkHeightField` take the visitor as `anytype` (a pointer); Jolt's local
//!   `struct Visitor : public CollideConvexVsTriangles` (and the other triangle helpers) embed the helper as `base`.
//!   `JPH_STACK_ALLOC` of the walker becomes fixed arrays for the largest block size (8).
//! - `HSGetTrianglesContext` keeps the output slices and the number of triangles found instead of Jolt's advancing
//!   `Float3 *` / `const PhysicsMaterial **` pointers.
//! - `RestoreMaterialState` allocates the material list (`Array::assign`), so `Shape.restoreMaterialState` returns
//!   `Allocator.Error!void` (the list uses the shape's allocator).
//! - `GetSubmergedVolume` is not supported (Jolt asserts): with asserts enabled it panics with Jolt's message, in a
//!   build without asserts it returns zeros (Jolt leaves the caller's out parameters untouched).
//! - Float to int conversions (`(int)floor(...)`, `(uint)floor(...)`) follow what Jolt's x86-64 build computes
//!   (cvttss2si): NaN and values outside the integer range give 0x80000000 (int) / 0 (uint), where Zig's
//!   `@intFromFloat` would be undefined. Values in range convert identically.
//! - Undefined behavior of Jolt that Zolt defines: a block size of 0 divides by zero while rounding the sample count
//!   (Zolt rounds to 0 and reports "Block size must be in the range [2, 8]!" like every other invalid block size), the
//!   shift `1 << mBitsPerSample` of an invalid number of bits per sample uses the x86 shift count (mod 32), and `clone`
//!   of a height field without collision (no buffers) does not copy from a null buffer (the clone has no collision
//!   either).
//! - Jolt behaviors that are kept as they are (found while comparing with Jolt, see the tests):
//!   - GetTrianglesNext restarts the block in which the output buffer became full, so triangles can be returned more
//!     than once, and a block with more triangles than requested (block size 5 or more with 32 triangles requested)
//!     is returned again and again: the walk never finishes.
//!   - An empty range block (min 0xffff, max 0) is skipped because its Y bounds are inside out. When
//!     offset.y + scale.y * 65535 == offset.y in float (a flat height field far from the origin) these bounds collapse
//!     to a valid box and the walker visits blocks outside the height field when the number of blocks is not a power
//!     of 2: Jolt reads beyond its grid and samples (undefined behavior), Zolt's safe builds assert like JPH_ASSERT.
//!     The same happens for a negative or zero scale.y in the settings.
//!   - Settings with more than one material and no material indices read beyond the material indices
//!     (StoreMaterialIndices), CalculateBitsPerSampleForError reads beyond the samples when the sample count is not a
//!     multiple of the block size: an index out of bounds in Zolt's safe builds.
//! - JPH_DEBUG_RENDERER (Draw, sDrawTriangleOutlines and the cached debug geometry) is not ported yet:
//!   TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const Color = @import("../../../Core/Color.zig").Color;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const TempAllocatorFile = @import("../../../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempAllocatorMalloc = TempAllocatorFile.TempAllocatorMalloc;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const AABox4 = @import("../../../Geometry/AABox4.zig");
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const RayInvDirection = RayAABox.RayInvDirection;
const rayAABox4 = RayAABox.rayAABox4;
const rayTriangle = @import("../../../Geometry/RayTriangle.zig").rayTriangle;
const math = @import("../../../Math/Math.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
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
const PhysicsMaterialList = ShapeFile.PhysicsMaterialList;
const PhysicsMaterialRefC = ShapeFile.PhysicsMaterialRefC;
const ConvexShape = @import("ConvexShape.zig").ConvexShape;
const SphereShape = @import("SphereShape.zig").SphereShape;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const BackFaceMode = @import("../BackFaceMode.zig").BackFaceMode;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const ActiveEdges = @import("../ActiveEdges.zig");
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;
const SortReverseAndStore = @import("../SortReverseAndStore.zig");
const sortReverseAndStore = SortReverseAndStore.sortReverseAndStore;
const countAndSortTrues = SortReverseAndStore.countAndSortTrues;
const CollideConvexVsTriangles = @import("../CollideConvexVsTriangles.zig").CollideConvexVsTriangles;
const CollideSphereVsTriangles = @import("../CollideSphereVsTriangles.zig").CollideSphereVsTriangles;
const CastConvexVsTriangles = @import("../CastConvexVsTriangles.zig").CastConvexVsTriangles;
const CastSphereVsTriangles = @import("../CastSphereVsTriangles.zig").CastSphereVsTriangles;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const CollideSoftBodyVerticesVsTriangles = @import("../CollideSoftBodyVerticesVsTriangles.zig").CollideSoftBodyVerticesVsTriangles;

/// Constants for HeightFieldShape, this was moved out of the HeightFieldShape because of a linker bug
pub const HeightFieldShapeConstants = struct {
    /// Value used to create gaps in the height field
    pub const no_collision_value: f32 = math.flt_max;

    /// Stack size to use during WalkHeightField
    pub const stack_size = 128;

    /// A position in the hierarchical grid is defined by a level (which grid), x and y position. We encode this in a single uint32 as: level << 28 | y << 14 | x
    pub const num_bits_xy: u32 = 14;
    pub const mask_bits_xy: u32 = (1 << HeightFieldShapeConstants.num_bits_xy) - 1;
    pub const level_shift: u32 = 2 * HeightFieldShapeConstants.num_bits_xy;

    /// When height samples are converted to 16 bit:
    /// This is the magic value for 'no collision'
    pub const no_collision_value16: u16 = 0xffff;
    /// This is the maximum allowed height value
    pub const max_height_value16: u16 = 0xfffe;

    /// Maximum value for HeightFieldShapeSettings::mBitsPerSample
    pub const max_bits_per_sample: u32 = 16;
};

const no_collision_value = HeightFieldShapeConstants.no_collision_value;
const stack_size = HeightFieldShapeConstants.stack_size;
const num_bits_xy = HeightFieldShapeConstants.num_bits_xy;
const mask_bits_xy = HeightFieldShapeConstants.mask_bits_xy;
const level_shift = HeightFieldShapeConstants.level_shift;
const no_collision_value16 = HeightFieldShapeConstants.no_collision_value16;
const max_height_value16 = HeightFieldShapeConstants.max_height_value16;
const max_bits_per_sample = HeightFieldShapeConstants.max_bits_per_sample;

/// Default value of the active edge cos threshold angle: cos(5 degrees)
const default_active_edge_cos_threshold_angle: f32 = 0.996195;

/// `(int)inValue` of a float as Jolt's x86-64 build computes it (cvttss2si): truncate, NaN and values outside the range
/// of an int give 0x80000000 (Zig's @intFromFloat is undefined for them)
fn truncToInt(value: f32) i32 {
    if (value >= -2147483648.0 and value < 2147483648.0)
        return @intFromFloat(value);
    return std.math.minInt(i32);
}

/// `(uint)inValue` of a float as Jolt's x86-64 build computes it (a 64 bit cvttss2si of which the low 32 bits are
/// used): NaN and values outside the range of an int64 give 0
fn truncToUint(value: f32) u32 {
    if (value >= -9223372036854775808.0 and value < 9223372036854775808.0)
        return @truncate(@as(u64, @bitCast(@as(i64, @intFromFloat(value)))));
    return 0;
}

/// `inPtr + inOffset` with a signed offset (strides can be negative)
fn offsetPtr(ptr: anytype, offset: isize) @TypeOf(ptr) {
    return if (offset >= 0) ptr + @as(usize, @intCast(offset)) else ptr - @as(usize, @intCast(-offset));
}

/// Jolt's `Array::resize(inNewSize, 0)` of a byte array: new elements are 0
fn resizeZeroFill(list: *std.ArrayList(u8), allocator: Allocator, new_len: usize) Allocator.Error!void {
    const old_len = list.items.len;
    try list.resize(allocator, new_len);
    if (new_len > old_len)
        @memset(list.items[old_len..], 0);
}

/// Replace the contents of `list` by references to `materials` (`list = materials` / `list.assign(...)`)
fn assignMaterials(list: *PhysicsMaterialList, allocator: Allocator, materials: []const PhysicsMaterialRefC) Allocator.Error!void {
    try list.ensureTotalCapacity(allocator, materials.len);
    for (list.items) |*m| m.deinit();
    list.clearRetainingCapacity();
    for (materials) |m|
        list.appendAssumeCapacity(m.clone());
}

/// Release the references in `list` and remove them (`list.clear()`, the capacity stays)
fn clearMaterials(list: *PhysicsMaterialList) void {
    for (list.items) |*m| m.deinit();
    list.clearRetainingCapacity();
}

/// Class that constructs a HeightFieldShape
pub const HeightFieldShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, HeightFieldShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    /// Optional arguments of the constructor (`const uint8 *inMaterialIndices = nullptr, const PhysicsMaterialList &inMaterialList = PhysicsMaterialList()`)
    pub const Options = struct {
        /// (sample_count - 1)^2 indices that index into `materials` (copied)
        material_indices: ?[]const u8 = null,
        /// The material list (a reference is added to each material)
        materials: []const *const PhysicsMaterial = &.{},
    };

    /// Result of `determineMinAndMaxSample` (Jolt's out parameters)
    pub const MinMaxSample = struct {
        /// The minimal value of height_samples or FLT_MAX if no samples have collision
        min_value: f32,
        /// The maximal value of height_samples or -FLT_MAX if no samples have collision
        max_value: f32,
        /// (value - min_value) * quantization_scale quantizes a height sample to 16 bits
        quantization_scale: f32,
    };

    base: ShapeSettings,

    /// The height field is a surface defined by: offset + scale * (x, height_samples[y * sample_count + x], y).
    /// where x and y are integers in the range x and y e [0, sample_count - 1].
    offset: Vec3 = Vec3.zero(),
    scale: Vec3 = Vec3.one(),
    sample_count: u32 = 0,

    /// Artificial minimal value of height_samples, used for compression and can be used to update the terrain after creating with lower height values. If there are any lower values in height_samples, this value will be ignored.
    min_height_value: f32 = math.large_float,

    /// Artificial maximum value of height_samples, used for compression and can be used to update the terrain after creating with higher height values. If there are any higher values in height_samples, this value will be ignored.
    max_height_value: f32 = -math.large_float,

    /// When bigger than materials.len the internal material list will be preallocated to support this number of materials.
    /// This avoids reallocations when calling HeightFieldShape::SetMaterials with new materials later.
    materials_capacity: u32 = 0,

    /// The heightfield is divided in blocks of block_size * block_size * 2 triangles and the acceleration structure culls blocks only,
    /// bigger block sizes reduce memory consumption but also reduce query performance. Valid values are [2, 8], does not need to be
    /// a power of 2. Note that at run-time we'll perform one more grid subdivision, so the effective block size is half of what is provided here.
    block_size: u32 = 2,

    /// How many bits per sample to use to compress the height field. Can be in the range [1, 16].
    /// Note that each sample is compressed relative to the min/max value of its block of block_size * block_size samples so the effective precision is higher.
    /// Also note that increasing block_size saves more memory than reducing the amount of bits per sample.
    bits_per_sample: u32 = 8,

    /// An array of sample_count^2 height samples. Samples are stored in row major order, so the sample at (x, y) is at index y * sample_count + x.
    /// (allocated with base.allocator)
    height_samples: std.ArrayList(f32) = .empty,

    /// An array of (sample_count - 1)^2 material indices. (allocated with base.allocator)
    material_indices: std.ArrayList(u8) = .empty,

    /// The materials of square at (x, y) is: materials[material_indices[x + y * (sample_count - 1)]]
    /// (allocated with base.allocator, holds a reference to each material)
    materials: PhysicsMaterialList = .empty,

    /// Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive).
    /// Setting this value too small can cause ghost collisions with edges, setting it too big can cause depenetration artifacts (objects not depenetrating quickly).
    /// Valid ranges are between cos(0 degrees) and cos(90 degrees). The default value is cos(5 degrees).
    active_edge_cos_threshold_angle: f32 = default_active_edge_cos_threshold_angle, // cos(5 degrees)

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) HeightFieldShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(HeightFieldShapeSettings), allocator) };
    }

    /// Create a height field shape of sample_count * sample_count vertices.
    /// The height field is a surface defined by: offset + scale * (x, samples[y * sample_count + x], y).
    /// where x and y are integers in the range x and y e [0, sample_count - 1].
    /// sample_count: sample_count / block_size must be minimally 2 and a power of 2 is the most efficient in terms of performance and storage.
    /// samples: sample_count^2 vertices.
    /// opts.material_indices: (sample_count - 1)^2 indices that index into opts.materials.
    /// Settings on the stack: `defer settings.deinit()`.
    pub fn init(allocator: Allocator, samples: []const f32, offset: Vec3, scale: Vec3, sample_count: u32, opts: Options) Allocator.Error!HeightFieldShapeSettings {
        var self = initDefault(allocator);
        errdefer self.deinit();
        self.offset = offset;
        self.scale = scale;
        self.sample_count = sample_count;

        try self.height_samples.appendSlice(allocator, samples[0 .. @as(usize, sample_count) * sample_count]);

        if (opts.materials.len != 0 and opts.material_indices != null) {
            const count_min_1: usize = sample_count - 1;
            try self.material_indices.appendSlice(allocator, opts.material_indices.?[0 .. count_min_1 * count_min_1]);
            try self.materials.ensureTotalCapacity(allocator, opts.materials.len);
            for (opts.materials) |m|
                self.materials.appendAssumeCapacity(.init(m));
        } else {
            std.debug.assert(opts.materials.len == 0);
            std.debug.assert(opts.material_indices == null);
        }

        return self;
    }

    /// new HeightFieldShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, samples: []const f32, offset: Vec3, scale: Vec3, sample_count: u32, opts: Options) Allocator.Error!*HeightFieldShapeSettings {
        const self = try allocator.create(HeightFieldShapeSettings);
        errdefer allocator.destroy(self);
        self.* = try init(allocator, samples, offset, scale, sample_count, opts);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *HeightFieldShapeSettings) *ShapeSettings {
        return &self.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *HeightFieldShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    /// ~HeightFieldShapeSettings: frees the arrays and releases the materials
    pub fn destruct(self: *HeightFieldShapeSettings) void {
        const allocator = self.base.allocator;
        self.height_samples.deinit(allocator);
        self.material_indices.deinit(allocator);
        clearMaterials(&self.materials);
        self.materials.deinit(allocator);
    }

    // See: ShapeSettings
    pub fn createShape(self: *HeightFieldShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(HeightFieldShape, self, allocator);
    }

    /// Determine the minimal and maximal value of height_samples (will ignore no_collision_value)
    /// @return The minimal value of height_samples or FLT_MAX if no samples have collision, the maximal value of
    /// height_samples or -FLT_MAX if no samples have collision and the scale with which (value - min_value) *
    /// quantization_scale quantizes a height sample to 16 bits
    pub fn determineMinAndMaxSample(self: *const HeightFieldShapeSettings) MinMaxSample {
        // Determine min and max value
        var min_value = self.min_height_value;
        var max_value = self.max_height_value;
        for (self.height_samples.items) |h|
            if (h != no_collision_value) {
                min_value = math.min(min_value, h);
                max_value = math.max(max_value, h);
            };

        // Prevent dividing by zero by setting a minimal height difference
        const height_diff = math.max(max_value - min_value, 1.0e-6);

        // Calculate the scale factor to quantize to 16 bits
        const quantization_scale = @as(f32, @floatFromInt(max_height_value16)) / height_diff;

        return .{ .min_value = min_value, .max_value = max_value, .quantization_scale = quantization_scale };
    }

    /// Given block_size, sample_count and height_samples, calculate the amount of bits needed to stay below absolute error max_error
    /// @param max_error Maximum allowed error in height_samples after compression (note that this does not take scale.Y into account)
    /// @return Needed bits per sample in the range [1, 16].
    /// Note: like Jolt this reads beyond the samples of the last row of blocks when sample_count is not a multiple of
    /// block_size (an index out of bounds in Zolt's safe builds).
    pub fn calculateBitsPerSampleForError(self: *const HeightFieldShapeSettings, max_error: f32) u32 {
        // Start with 1 bit per sample
        var bits_per_sample: u32 = 1;

        // Determine total range
        const range = self.determineMinAndMaxSample();
        const min_value = range.min_value;
        const max_value = range.max_value;
        const scale = range.quantization_scale;
        if (min_value < max_value) {
            // Loop over all blocks
            var y: u32 = 0;
            while (y < self.sample_count) : (y += self.block_size) {
                var x: u32 = 0;
                while (x < self.sample_count) : (x += self.block_size) {
                    // Determine min and max block value + take 1 sample border just like we do while building the hierarchical grids
                    var block_min_value: f32 = math.flt_max;
                    var block_max_value: f32 = -math.flt_max;
                    var bx = x;
                    while (bx < @min(x + self.block_size + 1, self.sample_count)) : (bx += 1) {
                        var by = y;
                        while (by < @min(y + self.block_size + 1, self.sample_count)) : (by += 1) {
                            const h = self.height_samples.items[by * self.sample_count + bx];
                            if (h != no_collision_value) {
                                block_min_value = math.min(block_min_value, h);
                                block_max_value = math.max(block_max_value, h);
                            }
                        }
                    }

                    if (block_min_value < block_max_value) {
                        // Quantize then dequantize block min/max value
                        block_min_value = min_value + @floor((block_min_value - min_value) * scale) / scale;
                        block_max_value = min_value + @ceil((block_max_value - min_value) * scale) / scale;
                        const block_height = block_max_value - block_min_value;

                        // Loop over the block again
                        bx = x;
                        while (bx < x + self.block_size) : (bx += 1) {
                            var by = y;
                            while (by < y + self.block_size) : (by += 1) {
                                // Get the height
                                const height = self.height_samples.items[by * self.sample_count + bx];
                                if (height != no_collision_value) {
                                    while (true) {
                                        // Determine bitmask for sample
                                        const sample_mask: u32 = (@as(u32, 1) << @intCast(bits_per_sample)) - 1;

                                        // Quantize
                                        var quantized_height = @floor((height - block_min_value) * @as(f32, @floatFromInt(sample_mask)) / block_height);
                                        quantized_height = math.clamp(quantized_height, 0.0, @as(f32, @floatFromInt(sample_mask - 1)));

                                        // Dequantize and check error
                                        const dequantized_height = block_min_value + (quantized_height + 0.5) * block_height / @as(f32, @floatFromInt(sample_mask));
                                        if (@abs(dequantized_height - height) <= max_error)
                                            break;

                                        // Not accurate enough, increase bits per sample
                                        bits_per_sample += 1;

                                        // Don't go above max_bits_per_sample bits per sample
                                        if (bits_per_sample == max_bits_per_sample)
                                            return max_bits_per_sample;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        return bits_per_sample;
    }
};

/// A height field shape. Cannot be used as a dynamic object.
///
/// Note: If you're using HeightFieldShape and are querying data while modifying the shape you'll have a race condition.
/// In this case it is best to create a new HeightFieldShape using the Clone function. You replace the shape on a body using BodyInterface::SetShape.
/// If a query is still working on the old shape, it will have taken a reference and keep the old shape alive until the query finishes.
pub const HeightFieldShape = struct {
    /// Concrete class: `Shape.cast(HeightFieldShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .height_field;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .mustBeStatic, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .saveMaterialState, .restoreMaterialState, .getStats, .getVolume, .restoreBinaryState };

    /// A block of 2x2 ranges used to form a hierarchical grid, ordered left top, right top, left bottom, right bottom
    pub const RangeBlock = extern struct {
        min: [4]u16 align(16),
        max: [4]u16,

        comptime {
            std.debug.assert(@sizeOf(RangeBlock) == 16 and @alignOf(RangeBlock) == 16);
        }
    };

    /// Offset of first RangedBlock in grid per level
    const grid_offsets = [_]u32{
        0, // level:  0, max x/y:     0, offset: 0
        1, // level:  1, max x/y:     1, offset: 1
        5, // level:  2, max x/y:     3, offset: 1 + 4
        21, // level:  3, max x/y:     7, offset: 1 + 4 + 16
        85, // level:  4, max x/y:    15, offset: 1 + 4 + 16 + 64
        341, // level:  5, max x/y:    31, offset: 1 + 4 + 16 + 64 + 256
        1365, // level:  6, max x/y:    63, offset: 1 + 4 + 16 + 64 + 256 + 1024
        5461, // level:  7, max x/y:   127, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096
        21845, // level:  8, max x/y:   255, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
        87381, // level:  9, max x/y:   511, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
        349525, // level: 10, max x/y:  1023, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
        1398101, // level: 11, max x/y:  2047, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
        5592405, // level: 12, max x/y:  4095, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
        22369621, // level: 13, max x/y:  8191, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
        89478485, // level: 14, max x/y: 16383, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
    };

    comptime {
        std.debug.assert(grid_offsets.len == num_bits_xy + 1); // Offsets array is not long enough
    }

    /// Largest block size (HeightFieldShapeSettings::mBlockSize is in the range [2, 8])
    const max_block_size = 8;

    base: Shape,

    /// The height field is a surface defined by: offset + scale * (x, height_samples[y * sample_count + x], y).
    /// where x and y are integers in the range x and y e [0, sample_count - 1].
    offset: Vec3 = Vec3.zero(),
    scale: Vec3 = Vec3.one(),

    // Height data

    /// See HeightFieldShapeSettings::mSampleCount
    sample_count: u32 = 0,
    /// See HeightFieldShapeSettings::mBlockSize
    block_size: u32 = 2,
    /// Size of height_samples in bytes
    height_samples_size: u32 = 0,
    /// Size of range_blocks in elements
    range_blocks_size: u32 = 0,
    /// Size of active_edges in bytes
    active_edges_size: u32 = 0,
    /// See HeightFieldShapeSettings::mBitsPerSample
    bits_per_sample: u8 = 8,
    /// All bits set for a sample: (1 << bits_per_sample) - 1, used to indicate that there's no collision
    sample_mask: u16 = 0xff,
    /// Min and max value in height_samples quantized to 16 bit, for calculating bounding box
    min_sample: u16 = no_collision_value16,
    max_sample: u16 = no_collision_value16,
    /// The single allocation that holds range_blocks, height_samples and active_edges (null when the height field has
    /// no collision; allocated with the shape's allocator)
    buffer: ?[]align(16) u8 = null,
    /// Hierarchical grid of range data describing the height variations within 1 block. The grid for level <level> starts at offset grid_offsets[<level>]
    range_blocks: []RangeBlock = &.{},
    /// bits_per_sample-bit height samples. Value [0, max_height_value] maps to highest detail grid in range_blocks [min, max]. sample_mask is reserved to indicate no collision.
    height_samples: []u8 = &.{},
    /// (sample_count - 1)^2 * 3-bit active edge flags.
    active_edges: []u8 = &.{},

    // Materials

    /// The materials of square at (x, y) is: materials[material_indices[x + y * (sample_count - 1)]]
    /// (allocated with the shape's allocator, holds a reference to each material)
    materials: PhysicsMaterialList = .empty,
    /// Compressed to the minimum amount of bits per material index (sample_count - 1) * (sample_count - 1) * num_bits_per_material_index bits of data
    /// (allocated with the shape's allocator)
    material_indices: std.ArrayList(u8) = .empty,
    /// Number of bits per material index
    num_bits_per_material_index: u32 = 0,

    // TODO(debug_renderer): mGeometry, mCachedUseMaterialColors (temporary rendering data), sDrawTriangleOutlines

    /// Result of getSubShapeCoordinates (Jolt's out parameters)
    pub const SubShapeCoordinates = struct {
        /// X coordinate of the triangle (in the range [0, sample_count - 2])
        x: u32,
        /// Y coordinate of the triangle (in the range [0, sample_count - 2])
        y: u32,
        /// Triangle within the quad (0 = lower triangle or 1 = upper triangle)
        triangle_index: u32,
    };

    /// Result of projectOntoSurface (Jolt's out parameters)
    pub const SurfacePosition = struct {
        /// The position on the surface (in the space of the shape)
        position: Vec3,
        /// The sub shape ID of the triangle that contains the position
        sub_shape_id: SubShapeID,
    };

    /// Optional arguments of setHeights
    pub const SetHeightsOptions = struct {
        /// Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive).
        active_edge_cos_threshold_angle: f32 = default_active_edge_cos_threshold_angle,
    };

    /// Result of getPositionInBlock (Jolt's private GetPosition overload with the out parameter outNoCollision)
    const PositionInBlock = struct {
        position: Vec3,
        no_collision: bool,
    };

    /// Result of getBlockOffsetAndScale (Jolt's out parameters)
    const BlockOffsetAndScale = struct {
        offset: f32,
        scale: f32,
    };

    /// Result of getRangeBlockOffsetAndStride (Jolt's out parameters)
    const RangeBlockOffsetAndStride = struct {
        offset: u32,
        stride: u32,
    };

    /// Result of getRangeBlock (Jolt's out parameters)
    const RangeBlockAndIndex = struct {
        block: *RangeBlock,
        index_in_block: u32,
    };

    /// Result of decodeSubShapeID (Jolt's out parameters)
    const DecodedSubShapeID = struct {
        x: u32,
        y: u32,
        triangle: u32,
    };

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// HeightFieldShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) HeightFieldShape {
        return .{ .base = .init(Shape.vtableFor(HeightFieldShape), allocator, .height_field, shape_sub_type) };
    }

    /// HeightFieldShape(const HeightFieldShapeSettings &inSettings, ShapeResult &outResult). `allocator` is used for the
    /// temporary data of the construction, the shape's own arrays use the shape's allocator.
    pub fn initFromSettings(self: *HeightFieldShape, settings: *const HeightFieldShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base);
        self.offset = settings.offset;
        self.scale = settings.scale;
        // Round sample count to nearest block size (Jolt divides by zero for a block size of 0, Zolt rounds to 0 and reports the invalid block size below)
        self.sample_count = if (settings.block_size != 0) ((settings.sample_count +% settings.block_size -% 1) / settings.block_size) *% settings.block_size else 0;
        self.block_size = settings.block_size;
        self.bits_per_sample = @truncate(settings.bits_per_sample);

        self.cacheValues();

        // Reserve a bigger materials list if requested
        if (settings.materials_capacity > 0)
            try self.materials.ensureTotalCapacity(self.base.allocator, settings.materials_capacity);
        try assignMaterials(&self.materials, self.base.allocator, settings.materials.items);

        // Check block size
        if (self.block_size < 2 or self.block_size > 8) {
            result.setError("HeightFieldShape: Block size must be in the range [2, 8]!");
            return;
        }

        // Check bits per sample
        if (settings.bits_per_sample < 1 or settings.bits_per_sample > max_bits_per_sample) {
            result.setError("HeightFieldShape: Bits per sample must be in the range [1, 16]!");
            return;
        }

        // We stop at block_size x block_size height sample blocks
        const num_blocks = self.getNumBlocks();

        // We want at least 1 grid layer
        if (num_blocks < 2) {
            result.setError("HeightFieldShape: Sample count too low!");
            return;
        }

        // Check that we don't overflow our 32 bit 'properties'
        if (num_blocks > (1 << num_bits_xy)) {
            result.setError("HeightFieldShape: Sample count too high!");
            return;
        }

        // Check if we're not exceeding the amount of sub shape id bits
        if (self.getSubShapeIDBitsRecursive() > SubShapeID.max_bits) {
            result.setError("HeightFieldShape: Size exceeds the amount of available sub shape ID bits!");
            return;
        }

        if (self.materials.items.len != 0) {
            // Validate materials
            if (self.materials.items.len > 256) {
                result.setError("Supporting max 256 materials per height field");
                return;
            }
            for (settings.material_indices.items) |s|
                if (s >= self.materials.items.len) {
                    result.setErrorFmt("Material {d} is beyond material list (size: {d})", .{ s, @as(u32, @intCast(self.materials.items.len)) });
                    return;
                };
        } else {
            // No materials assigned, validate that no materials have been specified
            if (settings.material_indices.items.len != 0) {
                result.setError("No materials present, mMaterialIndices should be empty");
                return;
            }
        }

        // Determine range
        const range = settings.determineMinAndMaxSample();
        var min_value = range.min_value;
        const max_value = range.max_value;
        const scale = range.quantization_scale;
        if (min_value > max_value) {
            // If there is no collision with this heightmap, leave everything empty
            clearMaterials(&self.materials);
            result.set(.init(self.asShapeMut()));
            return;
        }

        // Allocate space for this shape
        try self.allocateBuffers();

        // Quantize to uint16
        var quantized_samples: std.ArrayList(u16) = .empty;
        defer quantized_samples.deinit(allocator);
        try quantized_samples.ensureTotalCapacity(allocator, @as(usize, self.sample_count) * self.sample_count);
        for (0..settings.sample_count) |y| {
            for (0..settings.sample_count) |x| {
                const h = settings.height_samples.items[x + y * settings.sample_count];
                if (h == no_collision_value) {
                    quantized_samples.appendAssumeCapacity(no_collision_value16);
                } else {
                    // Floor the quantized height to get a lower bound for the quantized value
                    var quantized_height = truncToInt(@floor(scale * (h - min_value)));

                    // Ensure that the height says below the max height value so we can safely add 1 to get the upper bound for the quantized value
                    quantized_height = math.clamp(quantized_height, 0, @as(i32, max_height_value16 - 1));

                    quantized_samples.appendAssumeCapacity(@intCast(quantized_height));
                }
            }
            // Pad remaining columns with no collision
            for (settings.sample_count..self.sample_count) |_|
                quantized_samples.appendAssumeCapacity(no_collision_value16);
        }
        // Pad remaining rows with no collision
        for (settings.sample_count..self.sample_count) |_|
            for (0..self.sample_count) |_|
                quantized_samples.appendAssumeCapacity(no_collision_value16);

        // Update offset and scale to account for the compression to uint16
        if (min_value <= max_value) // Only when there was collision
        {
            // In GetPosition we always add 0.5 to the quantized sample in order to reduce the average error.
            // We want to be able to exactly quantize min_value (this is important in case the heightfield is entirely flat) so we subtract that value from min_value.
            min_value -= 0.5 / (scale * @as(f32, @floatFromInt(self.sample_mask)));

            self.offset.setY(self.offset.getY() + self.scale.getY() * min_value);
        }
        self.scale.setY(self.scale.getY() / scale);

        // Calculate amount of grids
        const max_level = getMaxLevel(num_blocks);

        // Temporary data structure used during creating of a hierarchy of grids
        const Range = struct {
            min: u16,
            max: u16,
        };

        // Reserve size for temporary range data + reserve 1 extra for a 1x1 grid that we won't store but use for calculating the bounding box.
        // ranges[level] is the grid of (1 << level) x (1 << level) ranges.
        var ranges: [num_bits_xy + 2][]Range = @splat(&.{});
        defer for (ranges[0 .. max_level + 1]) |r| allocator.free(r);

        // Calculate highest detail grid by combining block_size x block_size height samples
        const num_blocks_pow2 = math.getNextPowerOf2(num_blocks); // We calculate the range blocks as if the heightfield was a power of 2, when we save the range blocks we'll ignore the extra samples (this makes downsampling easier)
        std.debug.assert(num_blocks_pow2 == @as(u32, 1) << @intCast(max_level));
        ranges[max_level] = try allocator.alloc(Range, @as(usize, num_blocks_pow2) * num_blocks_pow2);
        {
            var range_dst: usize = 0;
            for (0..num_blocks_pow2) |y| {
                for (0..num_blocks_pow2) |x| {
                    const dst = &ranges[max_level][range_dst];
                    dst.min = 0xffff;
                    dst.max = 0;
                    const max_bx = if (x == num_blocks_pow2 - 1) self.block_size else self.block_size + 1; // for interior blocks take 1 more because the triangles connect to the next block so we must include their height too
                    const max_by = if (y == num_blocks_pow2 - 1) self.block_size else self.block_size + 1;
                    for (0..max_by) |by| {
                        for (0..max_bx) |bx| {
                            const sx = x * self.block_size + bx;
                            const sy = y * self.block_size + by;
                            if (sx < self.sample_count and sy < self.sample_count) {
                                const h = quantized_samples.items[sy * self.sample_count + sx];
                                if (h != no_collision_value16) {
                                    dst.min = @min(dst.min, h);
                                    dst.max = @max(dst.max, h + 1); // Add 1 to the max so we know the real value is between min and max
                                }
                            }
                        }
                    }
                    range_dst += 1;
                }
            }
        }

        // Calculate remaining grids
        {
            var level = max_level;
            var n = num_blocks_pow2 >> 1;
            while (n >= 1) : (n >>= 1) {
                // Get source buffer
                const range_src = ranges[level];

                // Previous array element
                level -= 1;

                // Make space for this grid
                ranges[level] = try allocator.alloc(Range, @as(usize, n) * n);

                // Get target buffer
                var range_dst: usize = 0;

                // Combine the results of 2x2 ranges
                for (0..n) |y| {
                    for (0..n) |x| {
                        const dst = &ranges[level][range_dst];
                        dst.min = 0xffff;
                        dst.max = 0;
                        for (0..2) |by| {
                            for (0..2) |bx| {
                                const r = range_src[(y * 2 + by) * n * 2 + x * 2 + bx];
                                dst.min = @min(dst.min, r.min);
                                dst.max = @max(dst.max, r.max);
                            }
                        }
                        range_dst += 1;
                    }
                }
            }
            std.debug.assert(level == 0);
        }

        // Store global range for bounding box calculation
        self.min_sample = ranges[0][0].min;
        self.max_sample = ranges[0][0].max;

        if (Core.enable_asserts) {
            // Validate that we did not lose range along the way
            var minv: u16 = 0xffff;
            var maxv: u16 = 0;
            for (quantized_samples.items) |v|
                if (v != no_collision_value16) {
                    minv = @min(minv, v);
                    maxv = @max(maxv, v + 1);
                };
            std.debug.assert(self.min_sample == minv and self.max_sample == maxv);
        }

        // Now erase the first element, we need a 2x2 grid to start with (Zolt: the grid of a level is ranges[level + 1])

        // Create blocks
        const max_stride = (num_blocks + 1) >> 1;
        var current_block: usize = 0;
        for (0..max_level) |level| {
            std.debug.assert(current_block == grid_offsets[level]);

            const level_ranges = ranges[level + 1];
            const in_n = @as(usize, 1) << @intCast(level);
            const out_n = @min(in_n, max_stride); // At the most detailed level we store a non-power of 2 number of blocks

            for (0..out_n) |y| {
                for (0..out_n) |x| {
                    // Convert from 2x2 Range structure to 1 RangeBlock structure
                    const rb = &self.range_blocks[current_block];
                    current_block += 1;
                    for (0..2) |by| {
                        for (0..2) |bx| {
                            const src_pos = (y * 2 + by) * 2 * in_n + (x * 2 + bx);
                            const dst_pos = by * 2 + bx;
                            rb.min[dst_pos] = level_ranges[src_pos].min;
                            rb.max[dst_pos] = level_ranges[src_pos].max;
                        }
                    }
                }
            }
        }
        std.debug.assert(current_block == self.range_blocks_size);

        // Quantize height samples
        @memset(self.height_samples, 0);
        var sample: u32 = 0;
        for (0..self.sample_count) |y| {
            for (0..self.sample_count) |x| {
                var output_value: u32 = undefined;

                const h = if (x < settings.sample_count and y < settings.sample_count) settings.height_samples.items[x + y * settings.sample_count] else no_collision_value;
                if (h == no_collision_value) {
                    // No collision
                    output_value = self.sample_mask;
                } else {
                    // Get range of block so we know what range to compress to
                    const bx = x / self.block_size;
                    const by = y / self.block_size;
                    const r = ranges[max_level][by * num_blocks_pow2 + bx];
                    std.debug.assert(r.min < r.max);

                    // Quantize to bits_per_sample bits, note that sample_mask is reserved for indicating that there's no collision.
                    // We divide the range into sample_mask segments and use the mid points of these segments as the quantized values.
                    // This results in a lower error than if we had quantized our data using the lowest point of all these segments.
                    const h_min = min_value + @as(f32, @floatFromInt(r.min)) / scale;
                    const h_delta = @as(f32, @floatFromInt(@as(i32, r.max) - @as(i32, r.min))) / scale;
                    const quantized_height = @floor((h - h_min) * @as(f32, @floatFromInt(self.sample_mask)) / h_delta);
                    output_value = @intCast(math.clamp(truncToInt(quantized_height), 0, @as(i32, self.sample_mask) - 1)); // sample_mask is reserved as 'no collision value'
                }

                // Store the sample
                const byte_pos = sample >> 3;
                const bit_pos: u5 = @intCast(sample & 0b111);
                output_value <<= bit_pos;
                std.debug.assert(byte_pos + 2 < self.height_samples_size); // We read max 16 bits which could be spread out over 3 bytes
                self.height_samples[byte_pos] |= @truncate(output_value);
                self.height_samples[byte_pos + 1] |= @truncate(output_value >> 8);
                self.height_samples[byte_pos + 2] |= @truncate(output_value >> 16);
                sample += settings.bits_per_sample;
            }
        }

        // Calculate the active edges
        try self.calculateActiveEdgesFromSettings(settings, allocator);

        // Compress material indices
        if (self.materials.items.len > 1 or settings.materials_capacity > 1)
            try self.storeMaterialIndices(settings);

        result.set(.init(self.asShapeMut()));
    }

    /// ~HeightFieldShape: frees the buffers and releases the materials
    pub fn destruct(self: *HeightFieldShape) void {
        const allocator = self.base.allocator;
        if (self.buffer) |data|
            allocator.free(data);
        clearMaterials(&self.materials);
        self.materials.deinit(allocator);
        self.material_indices.deinit(allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const HeightFieldShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *HeightFieldShape) *Shape {
        return &self.base;
    }

    /// Clone this shape. Can be used to avoid race conditions. See the documentation of this class for more information.
    /// Returns the new shape with reference count 0 (Jolt returns a Ref<HeightFieldShape>): put it in a Ref. `allocator`
    /// becomes the allocator of the clone.
    pub fn clone(self: *const HeightFieldShape, allocator: Allocator) Allocator.Error!*HeightFieldShape {
        const result = try allocator.create(HeightFieldShape);
        result.* = .initDefault(allocator);
        errdefer result.asShapeMut().destroy();
        result.asShapeMut().setUserData(self.base.getUserData());

        result.offset = self.offset;
        result.scale = self.scale;
        result.sample_count = self.sample_count;
        result.block_size = self.block_size;
        result.bits_per_sample = self.bits_per_sample;
        result.sample_mask = self.sample_mask;
        result.min_sample = self.min_sample;
        result.max_sample = self.max_sample;

        // Jolt allocates and copies unconditionally, which reads from a null buffer when this height field has no collision
        if (self.buffer) |data| {
            try result.allocateBuffers();
            @memcpy(result.buffer.?, data); // Copy the entire buffer in 1 go
        }

        try result.materials.ensureTotalCapacity(allocator, self.materials.capacity); // Ensure we keep the capacity of the original
        try assignMaterials(&result.materials, allocator, self.materials.items);
        try result.material_indices.appendSlice(allocator, self.material_indices.items);
        result.num_bits_per_material_index = self.num_bits_per_material_index;

        // TODO(debug_renderer): clone->mGeometry = mGeometry; clone->mCachedUseMaterialColors = mCachedUseMaterialColors;

        return result;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get the size of the height field. Note that this will always be rounded up to the nearest multiple of GetBlockSize().
    pub fn getSampleCount(self: *const HeightFieldShape) u32 {
        return self.sample_count;
    }

    /// Get the size of a block
    pub fn getBlockSize(self: *const HeightFieldShape) u32 {
        return self.block_size;
    }

    /// Get the range of height values that this height field can encode. Can be used to determine the allowed range when setting the height values with SetHeights.
    pub fn getMinHeightValue(self: *const HeightFieldShape) f32 {
        return self.offset.getY();
    }

    pub fn getMaxHeightValue(self: *const HeightFieldShape) f32 {
        return self.offset.getY() + self.scale.getY() * @as(f32, @floatFromInt(max_height_value16));
    }

    /// Get the current list of materials, the indices returned by GetMaterials() will index into this list.
    pub fn getMaterialList(self: *const HeightFieldShape) []const PhysicsMaterialRefC {
        return self.materials.items;
    }

    /// Calculate commonly used values and store them in the shape
    fn cacheValues(self: *HeightFieldShape) void {
        // The shift count of an invalid number of bits per sample (> 31) is taken mod 32 like the x86 shift instruction does (undefined in C++)
        self.sample_mask = @truncate((@as(u32, 1) << @as(u5, @truncate(self.bits_per_sample))) - 1);
    }

    /// Allocate the range_blocks, height_samples and active_edges buffers as a single data block
    fn allocateBuffers(self: *HeightFieldShape) Allocator.Error!void {
        const num_blocks = self.getNumBlocks();
        const max_stride = (num_blocks + 1) >> 1;
        const range_blocks_size = grid_offsets[getMaxLevel(num_blocks) - 1] + max_stride * max_stride;
        const height_samples_size = (self.sample_count *% self.sample_count *% self.bits_per_sample +% 7) / 8 + 2; // Since we read 3 bytes per sample, we need 2 extra bytes of padding
        const active_edges_size = ((self.sample_count -% 1) *% (self.sample_count -% 1) *% 3 +% 7) / 8 + 1; // See explanation at HeightFieldShape::CalculateActiveEdges

        std.debug.assert(self.buffer == null);
        const range_blocks_bytes = @as(usize, range_blocks_size) * @sizeOf(RangeBlock);
        const data = try self.base.allocator.alignedAlloc(u8, .fromByteUnits(@alignOf(RangeBlock)), range_blocks_bytes + height_samples_size + active_edges_size);
        self.buffer = data;
        self.range_blocks_size = range_blocks_size;
        self.height_samples_size = height_samples_size;
        self.active_edges_size = active_edges_size;
        self.range_blocks = @as([*]RangeBlock, @ptrCast(data.ptr))[0..range_blocks_size];
        self.height_samples = data[range_blocks_bytes .. range_blocks_bytes + height_samples_size];
        self.active_edges = data[range_blocks_bytes + height_samples_size ..];
    }

    /// Calculate bit mask for all active edges in the heightfield for a specific region
    fn calculateActiveEdges(self: *HeightFieldShape, x0: u32, y0: u32, size_x: u32, size_y: u32, heights: [*]const f32, heights_start_x: u32, heights_start_y: u32, heights_stride: isize, heights_scale: f32, active_edge_cos_threshold_angle: f32, temp_allocator: TempAllocator) Allocator.Error!void {
        // Limit the block size so we don't allocate more than 64K memory from the temp allocator
        const block_size_x: u32 = @min(size_x, 44);
        const block_size_y: u32 = @min(size_y, 44);

        // Allocate temporary buffer for normals
        const normals_size: u32 = 2 * (block_size_x + 1) * (block_size_y + 1) * @sizeOf(Vec3);
        const normals_block = try temp_allocator.allocate(normals_size);
        defer temp_allocator.free(normals_block, normals_size);
        const normals: [*]Vec3 = @ptrCast(@alignCast(normals_block.?));

        // Height sample at (x, y) of the region
        const Heights = struct {
            fn at(h: [*]const f32, stride: isize, row: u32, column: u32) [*]const f32 {
                return offsetPtr(h, @as(isize, row) * stride + @as(isize, column));
            }
        };

        // Update the edges in blocks
        var block_y: u32 = 0;
        while (block_y < size_y) : (block_y += block_size_y) {
            var block_x: u32 = 0;
            while (block_x < size_x) : (block_x += block_size_x) {
                // Calculate the bottom right corner of the block
                const block_x_end = @min(block_x + block_size_x, size_x);
                const block_y_end = @min(block_y + block_size_y, size_y);

                // If we're not at the first block in x, we need one extra column of normals to the left
                var normals_x_start: u32 = undefined;
                var normals_x_skip: u32 = undefined;
                if (block_x > 0) {
                    normals_x_start = block_x - 1;
                    normals_x_skip = 2; // We need to skip over that extra column
                } else {
                    normals_x_start = 0;
                    normals_x_skip = 0;
                }

                // If we're not at the last block in y, we need one extra row of normals at the bottom
                const normals_y_end = if (block_y_end < size_y) block_y_end + 1 else size_y;

                // Calculate triangle normals and make normals zero for triangles that are missing
                var out_normal: usize = 0;
                var y = block_y;
                while (y < normals_y_end) : (y += 1) {
                    var x = normals_x_start;
                    while (x < block_x_end) : (x += 1) {
                        // Get height on diagonal
                        const height_samples = Heights.at(heights, heights_stride, y0 - heights_start_y + y, x0 - heights_start_x + x);
                        const x1y1_h = height_samples[0];
                        const x2y2_h = offsetPtr(height_samples, heights_stride + 1)[0];
                        if (x1y1_h != no_collision_value and x2y2_h != no_collision_value) {
                            // Calculate normal for lower left triangle (e.g. T1A)
                            const x1y2_h = offsetPtr(height_samples, heights_stride)[0];
                            if (x1y2_h != no_collision_value) {
                                const x2y2_minus_x1y2 = Vec3.init(self.scale.getX(), heights_scale * (x2y2_h - x1y2_h), 0);
                                const x1y1_minus_x1y2 = Vec3.init(0, heights_scale * (x1y1_h - x1y2_h), -self.scale.getZ());
                                normals[out_normal] = x2y2_minus_x1y2.cross(x1y1_minus_x1y2).normalized();
                            } else normals[out_normal] = Vec3.zero();

                            // Calculate normal for upper right triangle (e.g. T1B)
                            const x2y1_h = height_samples[1];
                            if (x2y1_h != no_collision_value) {
                                const x1y1_minus_x2y1 = Vec3.init(-self.scale.getX(), heights_scale * (x1y1_h - x2y1_h), 0);
                                const x2y2_minus_x2y1 = Vec3.init(0, heights_scale * (x2y2_h - x2y1_h), self.scale.getZ());
                                normals[out_normal + 1] = x1y1_minus_x2y1.cross(x2y2_minus_x2y1).normalized();
                            } else normals[out_normal + 1] = Vec3.zero();
                        } else {
                            normals[out_normal] = Vec3.zero();
                            normals[out_normal + 1] = Vec3.zero();
                        }

                        out_normal += 2;
                    }
                }

                // Number of vectors to skip to get to the next row of normals
                const normals_pitch = 2 * (block_x_end - normals_x_start);

                // Calculate active edges
                var in_normal: usize = 0;
                var global_bit_pos = 3 * ((y0 + block_y) * (self.sample_count - 1) + (x0 + block_x));
                y = block_y;
                while (y < block_y_end) : (y += 1) {
                    in_normal += normals_x_skip; // If we have an extra column to the left, skip it here, we'll read it with in_normal[-1] below

                    var x = block_x;
                    while (x < block_x_end) : (x += 1) {
                        // Get vertex heights
                        const height_samples = Heights.at(heights, heights_stride, y0 - heights_start_y + y, x0 - heights_start_x + x);
                        const x1y1_h = height_samples[0];
                        const x1y2_h = offsetPtr(height_samples, heights_stride)[0];
                        const x2y2_h = offsetPtr(height_samples, heights_stride + 1)[0];
                        const x1y1_valid = x1y1_h != no_collision_value;
                        const x1y2_valid = x1y2_h != no_collision_value;
                        const x2y2_valid = x2y2_h != no_collision_value;

                        // Calculate the edge flags (3 bits)
                        // See diagram in the next function for the edge numbering
                        var edge_mask: u16 = 0b111;
                        var edge_flags: u16 = 0;

                        // Edge 0
                        if (x == 0) {
                            edge_mask &= 0b110; // We need normal x - 1 which we didn't calculate, don't update this edge
                        } else if (x1y1_valid and x1y2_valid) {
                            const edge0_direction = Vec3.init(0, heights_scale * (x1y2_h - x1y1_h), self.scale.getZ());
                            if (ActiveEdges.isEdgeActive(normals[in_normal], normals[in_normal - 1], edge0_direction, active_edge_cos_threshold_angle))
                                edge_flags |= 0b001;
                        }

                        // Edge 1
                        if (y == size_y - 1) {
                            edge_mask &= 0b101; // We need normal y + 1 which we didn't calculate, don't update this edge
                        } else if (x1y2_valid and x2y2_valid) {
                            const edge1_direction = Vec3.init(self.scale.getX(), heights_scale * (x2y2_h - x1y2_h), 0);
                            if (ActiveEdges.isEdgeActive(normals[in_normal], normals[in_normal + normals_pitch + 1], edge1_direction, active_edge_cos_threshold_angle))
                                edge_flags |= 0b010;
                        }

                        // Edge 2
                        if (x1y1_valid and x2y2_valid) {
                            const edge2_direction = Vec3.init(-self.scale.getX(), heights_scale * (x1y1_h - x2y2_h), -self.scale.getZ());
                            if (ActiveEdges.isEdgeActive(normals[in_normal], normals[in_normal + 1], edge2_direction, active_edge_cos_threshold_angle))
                                edge_flags |= 0b100;
                        }

                        // Store the edge flags in the array
                        const byte_pos = global_bit_pos >> 3;
                        const bit_pos: u4 = @intCast(global_bit_pos & 0b111);
                        std.debug.assert(byte_pos < self.active_edges_size);
                        const edge_flags_ptr = self.active_edges[byte_pos..];
                        var combined_edge_flags: u16 = @as(u16, edge_flags_ptr[0]) | (@as(u16, edge_flags_ptr[1]) << 8);
                        combined_edge_flags &= ~(edge_mask << bit_pos);
                        combined_edge_flags |= edge_flags << bit_pos;
                        edge_flags_ptr[0] = @truncate(combined_edge_flags);
                        edge_flags_ptr[1] = @truncate(combined_edge_flags >> 8);

                        in_normal += 2;
                        global_bit_pos += 3;
                    }

                    global_bit_pos += 3 * (self.sample_count - 1 - (block_x_end - block_x));
                }
            }
        }
    }

    /// Calculate bit mask for all active edges in the heightfield (CalculateActiveEdges(const HeightFieldShapeSettings &))
    fn calculateActiveEdgesFromSettings(self: *HeightFieldShape, settings: *const HeightFieldShapeSettings, allocator: Allocator) Allocator.Error!void {
        //  Store active edges. The triangles are organized like this:
        //      x --->
        //
        //  y   +       +
        //      | \ T1B | \ T2B
        //  |  e0   e2  |   \
        //  |   | T1A \ | T2A \
        //  V   +--e1---+-------+
        //      | \ T3B | \ T4B
        //      |   \   |   \
        //      | T3A \ | T4A \
        //      +-------+-------+
        //  We store active edges e0 .. e2 as bits 0 .. 2.
        //  We store triangles horizontally then vertically (order T1A, T2A, T3A and T4A).
        //  The top edge and right edge of the heightfield are always active so we do not need to store them,
        //  therefore we only need to store (mSampleCount - 1)^2 * 3-bit
        //  The triangles T1B, T2B, T3B and T4B do not need to be stored, their active edges can be constructed from adjacent triangles.
        //  Add 1 byte padding so we can always read 1 uint16 to get the bits that cross an 8 bit boundary

        // Make all edges active (if sample_count is bigger than settings.sample_count we need to fill up the padding,
        // also edges at x = 0 and y = settings.sample_count - 1 are not updated)
        @memset(self.active_edges, 0xff);

        // Now clear the edges that are not active
        var temp_allocator = TempAllocatorMalloc.init(allocator);
        try self.calculateActiveEdges(0, 0, settings.sample_count - 1, settings.sample_count - 1, settings.height_samples.items.ptr, 0, 0, settings.sample_count, settings.scale.getY(), settings.active_edge_cos_threshold_angle, temp_allocator.tempAllocator());
    }

    /// Store material indices in the least amount of bits per index possible
    fn storeMaterialIndices(self: *HeightFieldShape, settings: *const HeightFieldShapeSettings) Allocator.Error!void {
        // We need to account for any rounding of the sample count to the nearest block size
        const in_count_min_1 = settings.sample_count - 1;
        const out_count_min_1 = self.sample_count - 1;

        self.num_bits_per_material_index = 32 - math.countLeadingZeros(@max(@as(u32, @intCast(self.materials.items.len)), settings.materials_capacity) -% 1);
        try resizeZeroFill(&self.material_indices, self.base.allocator, ((out_count_min_1 * out_count_min_1 * self.num_bits_per_material_index + 7) >> 3) + 1); // Add 1 byte so we don't read out of bounds when reading an uint16

        if (self.materials.items.len > 1) {
            for (0..out_count_min_1) |y| {
                for (0..out_count_min_1) |x| {
                    // Read material
                    var material_index: u16 = if (x < in_count_min_1 and y < in_count_min_1) settings.material_indices.items[x + y * in_count_min_1] else 0;

                    // Calculate byte and bit position where the material index needs to go
                    const sample_pos = x + y * out_count_min_1;
                    const bit_pos_full = sample_pos * self.num_bits_per_material_index;
                    const byte_pos = bit_pos_full >> 3;
                    const bit_pos: u4 = @intCast(bit_pos_full & 0b111);

                    // Write the material index
                    material_index <<= bit_pos;
                    std.debug.assert(byte_pos + 1 < self.material_indices.items.len);
                    self.material_indices.items[byte_pos] |= @truncate(material_index);
                    self.material_indices.items[byte_pos + 1] |= @truncate(material_index >> 8);
                }
            }
        }
    }

    /// Get the amount of horizontal/vertical blocks
    fn getNumBlocks(self: *const HeightFieldShape) u32 {
        return self.sample_count / self.block_size;
    }

    /// Get the maximum level (amount of grids) of the tree
    fn getMaxLevel(num_blocks: u32) u32 {
        return 32 - math.countLeadingZeros(num_blocks -% 1);
    }

    /// Get the range block offset and stride for GetBlockOffsetAndScale
    fn getRangeBlockOffsetAndStride(num_blocks: u32, max_level: u32) RangeBlockOffsetAndStride {
        return .{ .offset = grid_offsets[max_level - 1], .stride = (num_blocks + 1) >> 1 };
    }

    /// For block (block_x, block_y) get the range block and the entry in the range block
    fn getRangeBlock(self: *HeightFieldShape, block_x: u32, block_y: u32, range_block_offset: u32, range_block_stride: u32) RangeBlockAndIndex {
        std.debug.assert(block_x < self.getNumBlocks() and block_y < self.getNumBlocks());

        // Convert to location of range block
        const rbx = block_x >> 1;
        const rby = block_y >> 1;
        const index_in_block = ((block_y & 1) << 1) + (block_x & 1);

        const offset = range_block_offset + rby * range_block_stride + rbx;
        std.debug.assert(offset < self.range_blocks_size);
        return .{ .block = &self.range_blocks[offset], .index_in_block = index_in_block };
    }

    /// For block (block_x, block_y) get the offset and scale needed to decode a uint8 height sample to a uint16
    fn getBlockOffsetAndScale(self: *const HeightFieldShape, block_x: u32, block_y: u32, range_block_offset: u32, range_block_stride: u32) BlockOffsetAndScale {
        std.debug.assert(block_x < self.getNumBlocks() and block_y < self.getNumBlocks());

        // Convert to location of range block
        const rbx = block_x >> 1;
        const rby = block_y >> 1;
        const n = ((block_y & 1) << 1) + (block_x & 1);

        // Calculate offset and scale
        const offset = range_block_offset + rby * range_block_stride + rbx;
        std.debug.assert(offset < self.range_blocks_size);
        const block = &self.range_blocks[offset];
        return .{
            .offset = @floatFromInt(block.min[n]),
            .scale = @as(f32, @floatFromInt(@as(i32, block.max[n]) - @as(i32, block.min[n]))) / @as(f32, @floatFromInt(self.sample_mask)),
        };
    }

    /// Get the height sample at position (x, y)
    fn getHeightSample(self: *const HeightFieldShape, x: u32, y: u32) u16 {
        std.debug.assert(x < self.sample_count);
        std.debug.assert(y < self.sample_count);

        // Determine bit position of sample
        const sample = (y * self.sample_count + x) * @as(u32, self.bits_per_sample);
        const byte_pos = sample >> 3;
        const bit_pos: u5 = @intCast(sample & 0b111);

        // Fetch the height sample value
        std.debug.assert(byte_pos + 2 < self.height_samples_size); // We read max 16 bits which could be spread out over 3 bytes
        const height_samples = self.height_samples[byte_pos..];
        const height_sample: u32 = @as(u32, height_samples[0]) | (@as(u32, height_samples[1]) << 8) | (@as(u32, height_samples[2]) << 16);
        return @as(u16, @truncate(height_sample >> bit_pos)) & self.sample_mask;
    }

    /// Faster version of GetPosition when block offset and scale are already known (Jolt's GetPosition overload with outNoCollision)
    fn getPositionInBlock(self: *const HeightFieldShape, x: u32, y: u32, block_offset: f32, block_scale: f32) PositionInBlock {
        // Get quantized value
        const height_sample = self.getHeightSample(x, y);

        // Add 0.5 to the quantized value to minimize the error (see constructor)
        return .{
            .position = self.offset.add(self.scale.mul(Vec3.init(@floatFromInt(x), block_offset + (0.5 + @as(f32, @floatFromInt(height_sample))) * block_scale, @floatFromInt(y)))),
            .no_collision = height_sample == self.sample_mask,
        };
    }

    /// Get height field position at sampled location (x, y).
    /// where x and y are integers in the range x e [0, sample_count - 1] and y e [0, sample_count - 1].
    pub fn getPosition(self: *const HeightFieldShape, x: u32, y: u32) Vec3 {
        // Test if there are any samples
        if (self.height_samples_size == 0)
            return self.offset.add(self.scale.mul(Vec3.init(@floatFromInt(x), 0.0, @floatFromInt(y))));

        // Get block location
        const bx = x / self.block_size;
        const by = y / self.block_size;

        // Calculate offset and stride
        const num_blocks = self.getNumBlocks();
        const range_block = getRangeBlockOffsetAndStride(num_blocks, getMaxLevel(num_blocks));

        const block = self.getBlockOffsetAndScale(bx, by, range_block.offset, range_block.stride);

        return self.getPositionInBlock(x, y, block.offset, block.scale).position;
    }

    /// Check if height field at sampled location (x, y) has collision (has a hole or not)
    pub fn isNoCollision(self: *const HeightFieldShape, x: u32, y: u32) bool {
        return self.height_samples_size == 0 or self.getHeightSample(x, y) == self.sample_mask;
    }

    /// Projects local_position (a point in the space of the shape) along the Y axis onto the surface and returns it with the sub shape ID of the triangle.
    /// When there is no surface position (because of a hole or because the point is outside the heightfield) the function will return null.
    pub fn projectOntoSurface(self: *const HeightFieldShape, local_position: Vec3) ?SurfacePosition {
        // Check if we have collision
        if (self.height_samples_size == 0)
            return null;

        // Convert coordinate to integer space
        const integer_space = local_position.sub(self.offset).div(self.scale);

        // Get x coordinate and fraction
        var x_frac = integer_space.getX();
        if (x_frac < 0.0 or x_frac >= @as(f32, @floatFromInt(self.sample_count - 1)))
            return null;
        const x = truncToUint(@floor(x_frac));
        x_frac -= @floatFromInt(x);

        // Get y coordinate and fraction
        var y_frac = integer_space.getZ();
        if (y_frac < 0.0 or y_frac >= @as(f32, @floatFromInt(self.sample_count - 1)))
            return null;
        const y = truncToUint(@floor(y_frac));
        y_frac -= @floatFromInt(y);

        // If one of the diagonal points doesn't have collision, we don't have a height at this location
        if (self.isNoCollision(x, y) or self.isNoCollision(x + 1, y + 1))
            return null;

        if (y_frac >= x_frac) {
            // Left bottom triangle, test the 3rd point
            if (self.isNoCollision(x, y + 1))
                return null;

            // Interpolate height value
            const v1 = self.getPosition(x, y);
            const v2 = self.getPosition(x, y + 1);
            const v3 = self.getPosition(x + 1, y + 1);
            const creator: SubShapeIDCreator = .{};
            return .{
                .position = v1.add(v2.sub(v1).mulScalar(y_frac)).add(v3.sub(v2).mulScalar(x_frac)),
                .sub_shape_id = self.encodeSubShapeID(creator, x, y, 0),
            };
        } else {
            // Right top triangle, test the third point
            if (self.isNoCollision(x + 1, y))
                return null;

            // Interpolate height value
            const v1 = self.getPosition(x, y);
            const v2 = self.getPosition(x + 1, y + 1);
            const v3 = self.getPosition(x + 1, y);
            const creator: SubShapeIDCreator = .{};
            return .{
                .position = v1.add(v2.sub(v3).mulScalar(y_frac)).add(v3.sub(v1).mulScalar(x_frac)),
                .sub_shape_id = self.encodeSubShapeID(creator, x, y, 1),
            };
        }
    }

    /// Returns the coordinates of the triangle that a sub shape ID represents
    /// @param sub_shape_id The sub shape ID to decode
    /// @return X coordinate of the triangle (in the range [0, sample_count - 2]), Y coordinate of the triangle (in the range [0, sample_count - 2]) and the triangle within the quad (0 = lower triangle or 1 = upper triangle)
    pub fn getSubShapeCoordinates(self: *const HeightFieldShape, sub_shape_id: SubShapeID) SubShapeCoordinates {
        const d = self.decodeSubShapeID(sub_shape_id);
        return .{ .x = d.x, .y = d.y, .triangle_index = d.triangle };
    }

    /// Get the height values of a block of data.
    /// Note that the height values are decompressed so will be slightly different from what the shape was originally created with.
    /// @param x Start X position, must be a multiple of block_size and in the range [0, sample_count - 1]
    /// @param y Start Y position, must be a multiple of block_size and in the range [0, sample_count - 1]
    /// @param size_x Number of samples in X direction, must be a multiple of block_size and in the range [0, sample_count - x]
    /// @param size_y Number of samples in Y direction, must be a multiple of block_size and in the range [0, sample_count - y]
    /// @param out_heights Returned height values, must be at least size_x * size_y floats. Values are returned in x-major order and can be no_collision_value.
    /// @param heights_stride Stride in floats between two consecutive rows of out_heights (can be negative if the data is upside down).
    pub fn getHeights(self: *const HeightFieldShape, x: u32, y: u32, size_x: u32, size_y: u32, out_heights: [*]f32, heights_stride: isize) void {
        if (size_x == 0 or size_y == 0)
            return;

        std.debug.assert(x % self.block_size == 0 and y % self.block_size == 0);
        std.debug.assert(x < self.sample_count and y < self.sample_count);
        std.debug.assert(x + size_x <= self.sample_count and y + size_y <= self.sample_count);

        // Test if there are any samples
        if (self.height_samples_size == 0) {
            // No samples, return the offset
            const offset = self.offset.getY();
            var out = out_heights;
            for (0..size_y) |_| {
                for (0..size_x) |xi|
                    out[xi] = offset;
                out = offsetPtr(out, heights_stride);
            }
        } else {
            // Calculate offset and stride
            const num_blocks = self.getNumBlocks();
            const range_block = getRangeBlockOffsetAndStride(num_blocks, getMaxLevel(num_blocks));

            // Loop over blocks
            const block_start_x = x / self.block_size;
            const block_start_y = y / self.block_size;
            const num_blocks_x = size_x / self.block_size;
            const num_blocks_y = size_y / self.block_size;
            for (0..num_blocks_y) |block_y| {
                for (0..num_blocks_x) |block_x| {
                    // Get offset and scale for block
                    const block = self.getBlockOffsetAndScale(block_start_x + @as(u32, @intCast(block_x)), block_start_y + @as(u32, @intCast(block_y)), range_block.offset, range_block.stride);

                    // Adjust by global offset and scale
                    // Note: This is the math applied in GetPosition() written out to reduce calculations in the inner loop
                    const scale = block.scale * self.scale.getY();
                    const offset = self.offset.getY() + self.scale.getY() * block.offset + 0.5 * scale;

                    // Loop over samples in block
                    for (0..self.block_size) |sample_y| {
                        for (0..self.block_size) |sample_x| {
                            // Calculate output coordinate
                            const output_x: u32 = @intCast(block_x * self.block_size + sample_x);
                            const output_y: u32 = @intCast(block_y * self.block_size + sample_y);

                            // Get quantized value
                            const height_sample = self.getHeightSample(x + output_x, y + output_y);

                            // Dequantize
                            const h = if (height_sample != self.sample_mask) offset + @as(f32, @floatFromInt(height_sample)) * scale else no_collision_value;
                            offsetPtr(out_heights, @as(isize, output_y) * heights_stride + @as(isize, output_x))[0] = h;
                        }
                    }
                }
            }
        }
    }

    /// Set the height values of a block of data.
    /// Note that this requires decompressing and recompressing a border of size block_size in the negative x/y direction so will cause some precision loss.
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    /// @param x Start X position, must be a multiple of block_size and in the range [0, sample_count - 1]
    /// @param y Start Y position, must be a multiple of block_size and in the range [0, sample_count - 1]
    /// @param size_x Number of samples in X direction, must be a multiple of block_size and in the range [0, sample_count - x]
    /// @param size_y Number of samples in Y direction, must be a multiple of block_size and in the range [0, sample_count - y]
    /// @param heights_in The new height values to set, must be an array of size_x * size_y floats, can be no_collision_value. Values outside of the range [GetMinHeightValue(), GetMaxHeightValue()] will be clamped.
    /// @param heights_stride_in Stride in floats between two consecutive rows of heights_in (can be negative if the data is upside down).
    /// @param temp_allocator Allocator to use for temporary memory
    /// @param opts.active_edge_cos_threshold_angle Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive).
    pub fn setHeights(self: *HeightFieldShape, x: u32, y: u32, size_x: u32, size_y: u32, heights_in: [*]const f32, heights_stride_in: isize, temp_allocator: TempAllocator, opts: SetHeightsOptions) Allocator.Error!void {
        if (size_x == 0 or size_y == 0)
            return;

        std.debug.assert(self.height_samples_size > 0);
        std.debug.assert(x % self.block_size == 0 and y % self.block_size == 0);
        std.debug.assert(x < self.sample_count and y < self.sample_count);
        std.debug.assert(x + size_x <= self.sample_count and y + size_y <= self.sample_count);

        // If we have a block in negative x/y direction, we will affect its range so we need to take it into account
        var need_temp_heights = false;
        var affected_x = x;
        var affected_y = y;
        var affected_size_x = size_x;
        var affected_size_y = size_y;
        if (x > 0) {
            affected_x -= self.block_size;
            affected_size_x += self.block_size;
            need_temp_heights = true;
        }
        if (y > 0) {
            affected_y -= self.block_size;
            affected_size_y += self.block_size;
            need_temp_heights = true;
        }

        // If we have a block in positive x/y direction, our ranges are affected by it so we need to take it into account
        var heights_size_x = affected_size_x;
        var heights_size_y = affected_size_y;
        if (x + size_x < self.sample_count) {
            heights_size_x += self.block_size;
            need_temp_heights = true;
        }
        if (y + size_y < self.sample_count) {
            heights_size_y += self.block_size;
            need_temp_heights = true;
        }

        // Get heights for affected area
        var heights: [*]const f32 = undefined;
        var heights_stride: isize = undefined;
        var temp_heights: ?[*]f32 = undefined;
        const temp_heights_size: u32 = heights_size_x * heights_size_y * @sizeOf(f32);
        if (need_temp_heights) {
            // Fetch the surrounding height data (note we're forced to recompress this data with a potentially different range so there will be some precision loss here)
            const temp_block = try temp_allocator.allocate(temp_heights_size);
            const temp: [*]f32 = @ptrCast(@alignCast(temp_block.?));
            temp_heights = temp;
            heights = temp;
            heights_stride = heights_size_x;

            // We need to fill in the following areas:
            //
            // +-----------------+
            // |        2        |
            // |---+---------+---|
            // |   |         |   |
            // | 3 |    1    | 4 |
            // |   |         |   |
            // |---+---------+---|
            // |        5        |
            // +-----------------+
            //
            // 1. The area that is affected by the new heights (we just copy these)
            // 2-5. These areas are either needed to calculate the range of the affected blocks or they need to be recompressed with a different range
            const offset_x = x - affected_x;
            const offset_y = y - affected_y;

            // Area 2
            self.getHeights(affected_x, affected_y, heights_size_x, offset_y, temp, heights_size_x);
            const area3_start = temp + offset_y * heights_size_x;

            // Area 3
            self.getHeights(affected_x, y, offset_x, size_y, area3_start, heights_size_x);

            // Area 1
            var area1_start = area3_start + offset_x;
            var in_heights = heights_in;
            for (0..size_y) |_| {
                @memcpy(area1_start[0..size_x], in_heights[0..size_x]);
                area1_start += heights_size_x;
                in_heights = offsetPtr(in_heights, heights_stride_in);
            }

            // Area 4
            const area4_x = x + size_x;
            self.getHeights(area4_x, y, affected_x + heights_size_x - area4_x, size_y, area3_start + (area4_x - affected_x), heights_size_x);

            // Area 5
            const area5_y = y + size_y;
            const area5_start = temp + (area5_y - affected_y) * heights_size_x;
            self.getHeights(affected_x, area5_y, heights_size_x, affected_y + heights_size_y - area5_y, area5_start, heights_size_x);
        } else {
            // We can directly use the input buffer because there are no extra edges to take into account
            heights = heights_in;
            heights_stride = heights_stride_in;
            temp_heights = null;
        }

        // Calculate offset and stride
        var num_blocks = self.getNumBlocks();
        var max_level = getMaxLevel(num_blocks);
        const range_block = getRangeBlockOffsetAndStride(num_blocks, max_level);
        var range_block_offset = range_block.offset;
        var range_block_stride = range_block.stride;

        // Loop over blocks
        var block_start_x = affected_x / self.block_size;
        var block_start_y = affected_y / self.block_size;
        var num_blocks_x = affected_size_x / self.block_size;
        var num_blocks_y = affected_size_y / self.block_size;
        {
            var block_y: u32 = 0;
            var sample_start_y: u32 = 0;
            while (block_y < num_blocks_y) : ({
                block_y += 1;
                sample_start_y += self.block_size;
            }) {
                var block_x: u32 = 0;
                var sample_start_x: u32 = 0;
                while (block_x < num_blocks_x) : ({
                    block_x += 1;
                    sample_start_x += self.block_size;
                }) {
                    // Determine quantized min and max value for block
                    // Note that we need to include 1 extra row in the positive x/y direction to account for connecting triangles
                    var min_value: i32 = 0xffff;
                    var max_value: i32 = 0;
                    var sample_x_end = @min(sample_start_x + self.block_size + 1, self.sample_count - affected_x);
                    var sample_y_end = @min(sample_start_y + self.block_size + 1, self.sample_count - affected_y);
                    var sample_y = sample_start_y;
                    while (sample_y < sample_y_end) : (sample_y += 1) {
                        var sample_x = sample_start_x;
                        while (sample_x < sample_x_end) : (sample_x += 1) {
                            const h = offsetPtr(heights, @as(isize, sample_y) * heights_stride + @as(isize, sample_x))[0];
                            if (h != no_collision_value) {
                                const quantized_height = math.clamp(truncToInt(@floor((h - self.offset.getY()) / self.scale.getY())), 0, @as(i32, max_height_value16 - 1));
                                min_value = @min(min_value, quantized_height);
                                max_value = @max(max_value, quantized_height + 1);
                            }
                        }
                    }
                    if (min_value > max_value) {
                        min_value = no_collision_value16;
                        max_value = no_collision_value16;
                    }

                    // Update range for block
                    const rb = self.getRangeBlock(block_start_x + block_x, block_start_y + block_y, range_block_offset, range_block_stride);
                    rb.block.min[rb.index_in_block] = @intCast(min_value);
                    rb.block.max[rb.index_in_block] = @intCast(max_value);

                    // Get offset and scale for block
                    const offset_block: f32 = @floatFromInt(min_value);
                    const scale_block = @as(f32, @floatFromInt(max_value - min_value)) / @as(f32, @floatFromInt(self.sample_mask));

                    // Calculate scale and offset using the formula used in GetPosition() solved for the quantized height (excluding 0.5 because we round down while quantizing)
                    const scale = scale_block * self.scale.getY();
                    const offset = self.offset.getY() + offset_block * self.scale.getY();

                    // Loop over samples in block
                    sample_x_end = sample_start_x + self.block_size;
                    sample_y_end = sample_start_y + self.block_size;
                    sample_y = sample_start_y;
                    while (sample_y < sample_y_end) : (sample_y += 1) {
                        var sample_x = sample_start_x;
                        while (sample_x < sample_x_end) : (sample_x += 1) {
                            // Quantize height
                            const h = offsetPtr(heights, @as(isize, sample_y) * heights_stride + @as(isize, sample_x))[0];
                            const quantized_height: u16 = if (h != no_collision_value) @intCast(math.clamp(truncToInt(@floor((h - offset) / scale)), 0, @as(i32, self.sample_mask) - 1)) else self.sample_mask;

                            // Determine bit position of sample
                            const sample = ((affected_y + sample_y) * self.sample_count + affected_x + sample_x) * @as(u32, self.bits_per_sample);
                            const byte_pos = sample >> 3;
                            const bit_pos: u5 = @intCast(sample & 0b111);

                            // Update the height value sample
                            std.debug.assert(byte_pos + 2 < self.height_samples_size); // We read max 16 bits which could be spread out over 3 bytes
                            const height_samples = self.height_samples[byte_pos..];
                            var height_sample: u32 = @as(u32, height_samples[0]) | (@as(u32, height_samples[1]) << 8) | (@as(u32, height_samples[2]) << 16);
                            height_sample &= ~(@as(u32, self.sample_mask) << bit_pos);
                            height_sample |= @as(u32, quantized_height) << bit_pos;
                            height_samples[0] = @truncate(height_sample);
                            height_samples[1] = @truncate(height_sample >> 8);
                            height_samples[2] = @truncate(height_sample >> 16);
                        }
                    }
                }
            }
        }

        // Update active edges
        // Note that we must take an extra row on all sides to account for connecting triangles
        const ae_x = if (x > 1) x - 2 else 0;
        const ae_y = if (y > 1) y - 2 else 0;
        const ae_sx = @min(x + size_x + 1, self.sample_count - 1) - ae_x;
        const ae_sy = @min(y + size_y + 1, self.sample_count - 1) - ae_y;
        const active_edges_result = self.calculateActiveEdges(ae_x, ae_y, ae_sx, ae_sy, heights, affected_x, affected_y, heights_stride, 1.0, opts.active_edge_cos_threshold_angle, temp_allocator);

        // Free temporary buffer
        if (temp_heights) |temp|
            temp_allocator.free(temp, temp_heights_size);

        // Out of memory in the active edge calculation (Jolt aborts): the heights have been updated, the active edges
        // are partially updated
        try active_edges_result;

        // Update hierarchy of range blocks
        while (max_level > 1) {
            // Get offset and stride for destination blocks. Only the most detailed level is stored with a stride of
            // (num_blocks + 1) / 2, all coarser levels are stored with a stride of 1 << level (see the constructor and
            // WalkHeightField)
            const dst_range_block_offset = grid_offsets[max_level - 2];
            const dst_range_block_stride = @as(u32, 1) << @intCast(max_level - 2);

            // We'll be processing 2x2 blocks below so we need the start coordinates to be even and we extend the number of blocks to correct for that
            if ((block_start_x & 1) != 0) {
                block_start_x -= 1;
                num_blocks_x += 1;
            }
            if ((block_start_y & 1) != 0) {
                block_start_y -= 1;
                num_blocks_y += 1;
            }

            // Loop over all affected blocks
            const block_end_x = block_start_x + num_blocks_x;
            const block_end_y = block_start_y + num_blocks_y;
            var block_y = block_start_y;
            while (block_y < block_end_y) : (block_y += 2) {
                var block_x = block_start_x;
                while (block_x < block_end_x) : (block_x += 2) {
                    // Get source range block
                    const src = self.getRangeBlock(block_x, block_y, range_block_offset, range_block_stride);

                    // Determine quantized min and max value for the entire 2x2 block
                    var min_value: u16 = 0xffff;
                    var max_value: u16 = 0;
                    for (0..4) |i|
                        if (src.block.min[i] != no_collision_value16) {
                            min_value = @min(min_value, src.block.min[i]);
                            max_value = @max(max_value, src.block.max[i]);
                        };

                    // Write to destination block
                    const dst = self.getRangeBlock(block_x >> 1, block_y >> 1, dst_range_block_offset, dst_range_block_stride);
                    dst.block.min[dst.index_in_block] = min_value;
                    dst.block.max[dst.index_in_block] = max_value;
                }
            }

            // Go up one level
            max_level -= 1;
            num_blocks >>= 1;
            block_start_x >>= 1;
            block_start_y >>= 1;
            num_blocks_x = @min((num_blocks_x + 1) >> 1, num_blocks);
            num_blocks_y = @min((num_blocks_y + 1) >> 1, num_blocks);

            // Update stride and offset for source to old destination
            range_block_offset = dst_range_block_offset;
            range_block_stride = dst_range_block_stride;
        }

        // Calculate new min and max sample for the entire height field
        self.min_sample = 0xffff;
        self.max_sample = 0;
        for (0..4) |i|
            if (self.range_blocks[0].min[i] != no_collision_value16) {
                self.min_sample = @min(self.min_sample, self.range_blocks[0].min[i]);
                self.max_sample = @max(self.max_sample, self.range_blocks[0].max[i]);
            };

        // TODO(debug_renderer): Invalidate temporary rendering data (mGeometry.clear())
    }

    /// Get the material indices of a block of data.
    /// @param x Start X position, must in the range [0, sample_count - 1]
    /// @param y Start Y position, must in the range [0, sample_count - 1]
    /// @param size_x Number of samples in X direction
    /// @param size_y Number of samples in Y direction
    /// @param out_materials Returned material indices, must be at least size_x * size_y uint8s. Values are returned in x-major order.
    /// @param materials_stride Stride in uint8s between two consecutive rows of out_materials (can be negative if the data is upside down).
    pub fn getMaterials(self: *const HeightFieldShape, x: u32, y: u32, size_x: u32, size_y: u32, out_materials: [*]u8, materials_stride: isize) void {
        if (size_x == 0 or size_y == 0)
            return;

        if (self.material_indices.items.len == 0) {
            // Return all 0's
            for (0..size_y) |yi| {
                const out_indices = offsetPtr(out_materials, @as(isize, @intCast(yi)) * materials_stride);
                for (0..size_x) |xi|
                    out_indices[xi] = 0;
            }
            return;
        }

        std.debug.assert(x < self.sample_count and y < self.sample_count);
        std.debug.assert(x + size_x < self.sample_count and y + size_y < self.sample_count);

        const count_min_1 = self.sample_count - 1;
        const material_index_mask: u16 = @truncate((@as(u32, 1) << @intCast(self.num_bits_per_material_index)) - 1);

        for (0..size_y) |yi| {
            // Calculate input position
            var bit_pos = (x + (y + @as(u32, @intCast(yi))) * count_min_1) * self.num_bits_per_material_index;
            var in_indices: usize = bit_pos >> 3;
            bit_pos &= 0b111;

            // Calculate output position
            const out_indices = offsetPtr(out_materials, @as(isize, @intCast(yi)) * materials_stride);

            for (0..size_x) |xi| {
                // Get material index
                var material_index: u16 = @as(u16, self.material_indices.items[in_indices]) + (@as(u16, self.material_indices.items[in_indices + 1]) << 8);
                material_index >>= @intCast(bit_pos);
                material_index &= material_index_mask;
                out_indices[xi] = @truncate(material_index);

                // Go to the next index
                bit_pos += self.num_bits_per_material_index;
                in_indices += bit_pos >> 3;
                bit_pos &= 0b111;
            }
        }
    }

    /// Set the material indices of a block of data.
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    /// @param x Start X position, must in the range [0, sample_count - 1]
    /// @param y Start Y position, must in the range [0, sample_count - 1]
    /// @param size_x Number of samples in X direction
    /// @param size_y Number of samples in Y direction
    /// @param materials The new material indices, must be at least size_x * size_y uint8s. Values are returned in x-major order.
    /// @param materials_stride Stride in uint8s between two consecutive rows of materials (can be negative if the data is upside down).
    /// @param material_list The material list to use for the new material indices or null if the material list should not be updated
    /// @param temp_allocator Allocator to use for temporary memory
    /// @return True if the material indices were set, false if the total number of materials exceeded 256
    pub fn setMaterials(self: *HeightFieldShape, x: u32, y: u32, size_x: u32, size_y: u32, materials: [*]const u8, materials_stride: isize, material_list: ?[]const PhysicsMaterialRefC, temp_allocator: TempAllocator) Allocator.Error!bool {
        if (size_x == 0 or size_y == 0)
            return true;

        std.debug.assert(x < self.sample_count and y < self.sample_count);
        std.debug.assert(x + size_x < self.sample_count and y + size_y < self.sample_count);

        const allocator = self.base.allocator;

        // Remap materials
        const material_remap_table_size: u32 = @intCast(if (material_list) |list| list.len else self.materials.items.len);
        const remap_block = try temp_allocator.allocate(material_remap_table_size);
        defer temp_allocator.free(remap_block, material_remap_table_size);
        const material_remap_table: []u8 = if (remap_block) |b| b[0..material_remap_table_size] else &.{};
        if (material_list) |list| {
            // Conservatively reserve more space if the incoming material list is bigger
            if (list.len > self.materials.items.len)
                try self.materials.ensureTotalCapacity(allocator, list.len);

            // Create a remap table
            for (list, 0..) |material, remap_entry| {
                // Try to find it in the existing list
                var found: ?usize = null;
                for (self.materials.items, 0..) |m, i|
                    if (m.get() == material.get()) {
                        found = i;
                        break;
                    };
                if (found) |index| {
                    // Found it, calculate index
                    material_remap_table[remap_entry] = @intCast(index);
                } else {
                    // Not found, add it
                    if (self.materials.items.len >= 256) {
                        // We can't have more than 256 materials since we use uint8 as indices
                        return false;
                    }
                    material_remap_table[remap_entry] = @intCast(self.materials.items.len);
                    try self.materials.ensureUnusedCapacity(allocator, 1);
                    self.materials.appendAssumeCapacity(material.clone());
                }
            }
        } else {
            // No remapping
            for (material_remap_table, 0..) |*entry, i|
                entry.* = @intCast(i);
        }

        if (self.materials.items.len == 1) {
            // Only 1 material, we don't need to store the material indices
            return true;
        }

        // Check if we need to resize the material indices array
        const count_min_1 = self.sample_count - 1;
        const new_bits_per_material_index = 32 - math.countLeadingZeros(@as(u32, @intCast(self.materials.items.len)) -% 1);
        std.debug.assert(self.num_bits_per_material_index <= 8 and new_bits_per_material_index <= 8);
        if (new_bits_per_material_index > self.num_bits_per_material_index) {
            // Resize the material indices array
            try resizeZeroFill(&self.material_indices, allocator, ((count_min_1 * count_min_1 * new_bits_per_material_index + 7) >> 3) + 1); // Add 1 byte so we don't read out of bounds when reading an uint16

            // Calculate old and new mask
            const old_material_index_mask: u16 = @truncate((@as(u32, 1) << @intCast(self.num_bits_per_material_index)) - 1);
            const new_material_index_mask: u16 = @truncate((@as(u32, 1) << @intCast(new_bits_per_material_index)) - 1);

            // Loop through the array backwards to avoid overwriting data
            const data = self.material_indices.items;
            var in_bit_pos: i32 = @intCast((count_min_1 * count_min_1 - 1) * self.num_bits_per_material_index);
            var in_indices: isize = in_bit_pos >> 3;
            in_bit_pos &= 0b111;
            var out_bit_pos: i32 = @intCast((count_min_1 * count_min_1 - 1) * new_bits_per_material_index);
            var out_indices: isize = out_bit_pos >> 3;
            out_bit_pos &= 0b111;

            while (out_indices >= 0) {
                // Read the material index
                const in_index: usize = @intCast(in_indices);
                var material_index: u16 = @as(u16, data[in_index]) + (@as(u16, data[in_index + 1]) << 8);
                material_index >>= @intCast(in_bit_pos);
                material_index &= old_material_index_mask;

                // Write the material index
                const out_index: usize = @intCast(out_indices);
                var output_data: u16 = @as(u16, data[out_index]) + (@as(u16, data[out_index + 1]) << 8);
                output_data &= ~(new_material_index_mask << @intCast(out_bit_pos));
                output_data |= material_index << @intCast(out_bit_pos);
                data[out_index] = @truncate(output_data);
                data[out_index + 1] = @truncate(output_data >> 8);

                // Go to the previous index
                in_bit_pos -= @intCast(self.num_bits_per_material_index);
                in_indices += in_bit_pos >> 3;
                in_bit_pos &= 0b111;
                out_bit_pos -= @intCast(new_bits_per_material_index);
                out_indices += out_bit_pos >> 3;
                out_bit_pos &= 0b111;
            }

            // Accept the new bits per material index
            self.num_bits_per_material_index = new_bits_per_material_index;
        }

        const material_index_mask: u16 = @truncate((@as(u32, 1) << @intCast(self.num_bits_per_material_index)) - 1);
        for (0..size_y) |yi| {
            // Calculate input position
            const in_indices = offsetPtr(materials, @as(isize, @intCast(yi)) * materials_stride);

            // Calculate output position
            var bit_pos = (x + (y + @as(u32, @intCast(yi))) * count_min_1) * self.num_bits_per_material_index;
            var out_indices: usize = bit_pos >> 3;
            bit_pos &= 0b111;

            for (0..size_x) |xi| {
                // Update material
                const out = self.material_indices.items[out_indices..];
                var output_data: u16 = @as(u16, out[0]) + (@as(u16, out[1]) << 8);
                output_data &= ~(material_index_mask << @intCast(bit_pos));
                output_data |= @as(u16, material_remap_table[in_indices[xi]]) << @intCast(bit_pos);
                out[0] = @truncate(output_data);
                out[1] = @truncate(output_data >> 8);

                // Go to the next index
                bit_pos += self.num_bits_per_material_index;
                out_indices += bit_pos >> 3;
                bit_pos &= 0b111;
            }
        }

        return true;
    }

    /// Overload to get the material at a particular location (GetMaterial(uint inX, uint inY))
    pub fn getMaterialAt(self: *const HeightFieldShape, x: u32, y: u32) *const PhysicsMaterial {
        if (self.materials.items.len == 0)
            return PhysicsMaterial.default;
        if (self.materials.items.len == 1)
            return self.materials.items[0].get().?;

        const count_min_1 = self.sample_count - 1;
        std.debug.assert(x < count_min_1);
        std.debug.assert(y < count_min_1);

        // Calculate at which bit the material index starts
        var bit_pos = (x + y * count_min_1) * self.num_bits_per_material_index;
        const byte_pos = bit_pos >> 3;
        bit_pos &= 0b111;

        // Read the material index
        std.debug.assert(byte_pos + 1 < self.material_indices.items.len);
        const material_indices = self.material_indices.items[byte_pos..];
        var material_index: u16 = @as(u16, material_indices[0]) + (@as(u16, material_indices[1]) << 8);
        material_index >>= @intCast(bit_pos);
        material_index &= @truncate((@as(u32, 1) << @intCast(self.num_bits_per_material_index)) - 1);

        // Return the material
        return self.materials.items[material_index].get().?;
    }

    /// Determine amount of bits needed to encode sub shape id
    fn getSubShapeIDBits(self: *const HeightFieldShape) u32 {
        // Need to store X, Y and 1 extra bit to specify the triangle number in the quad
        return 2 * (32 - math.countLeadingZeros(self.sample_count -% 1)) + 1;
    }

    /// Encode a sub shape ID. x and y specify the coordinate of the triangle. triangle == 0 is the lower triangle, triangle == 1 is the upper triangle.
    fn encodeSubShapeID(self: *const HeightFieldShape, creator: SubShapeIDCreator, x: u32, y: u32, triangle: u32) SubShapeID {
        return creator.pushID((x + y * self.sample_count) * 2 + triangle, self.getSubShapeIDBits()).getID();
    }

    /// Decode a sub shape ID. x and y specify the coordinate of the triangle. triangle == 0 is the lower triangle, triangle == 1 is the upper triangle.
    fn decodeSubShapeID(self: *const HeightFieldShape, sub_shape_id: SubShapeID) DecodedSubShapeID {
        // Decode sub shape id
        const popped = sub_shape_id.popID(self.getSubShapeIDBits());
        if (Core.enable_asserts) std.debug.assert(popped.remainder.isEmpty()); // Invalid subshape ID (a wrong ID from the caller, Jolt's release build continues)
        var id = popped.id;

        // Get triangle index
        const triangle = id & 1;
        id >>= 1;

        // Fetch the x and y coordinate
        return .{ .x = id % self.sample_count, .y = id / self.sample_count, .triangle = triangle };
    }

    /// Get the edge flags for a triangle
    fn getEdgeFlags(self: *const HeightFieldShape, x: u32, y: u32, triangle: u32) u8 {
        std.debug.assert(x < self.sample_count - 1 and y < self.sample_count - 1);

        if (triangle == 0) {
            // The edge flags for this triangle are directly stored, find the right 3 bits
            var bit_pos = 3 * (x + y * (self.sample_count - 1));
            const byte_pos = bit_pos >> 3;
            bit_pos &= 0b111;
            std.debug.assert(byte_pos + 1 < self.active_edges_size);
            const active_edges = self.active_edges[byte_pos..];
            const edge_flags: u16 = @as(u16, active_edges[0]) + (@as(u16, active_edges[1]) << 8);
            return @as(u8, @truncate(edge_flags >> @intCast(bit_pos))) & 0b111;
        } else {
            // We don't store this triangle directly, we need to look at our three neighbours to construct the edge flags
            const edge0: u8 = if ((self.getEdgeFlags(x, y, 0) & 0b100) != 0) 0b001 else 0; // Diagonal edge
            const edge1: u8 = if (x == self.sample_count - 2 or (self.getEdgeFlags(x + 1, y, 0) & 0b001) != 0) 0b010 else 0; // Vertical edge
            const edge2: u8 = if (y == 0 or (self.getEdgeFlags(x, y - 1, 0) & 0b010) != 0) 0b100 else 0; // Horizontal edge
            return edge0 | edge1 | edge2;
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::MustBeStatic
    pub fn mustBeStatic(self: *const HeightFieldShape) bool {
        _ = self;
        return true;
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const HeightFieldShape) AABox {
        if (self.min_sample == no_collision_value16) {
            // This whole height field shape doesn't have any collision, return the center point
            const center = self.offset.add(self.scale.mulScalar(0.5).mul(Vec3.init(@floatFromInt(self.sample_count -% 1), 0.0, @floatFromInt(self.sample_count -% 1))));
            return .init(center, center);
        } else {
            // Bounding box based on min and max sample height
            const bmin = self.offset.add(self.scale.mul(Vec3.init(0.0, @floatFromInt(self.min_sample), 0.0)));
            const bmax = self.offset.add(self.scale.mul(Vec3.init(@floatFromInt(self.sample_count - 1), @floatFromInt(self.max_sample), @floatFromInt(self.sample_count - 1))));
            return .init(bmin, bmax);
        }
    }

    // See Shape::GetSubShapeIDBitsRecursive
    pub fn getSubShapeIDBitsRecursive(self: *const HeightFieldShape) u32 {
        return self.getSubShapeIDBits();
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const HeightFieldShape) f32 {
        _ = self;
        return 0.0;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const HeightFieldShape) MassProperties {
        _ = self;
        // Object should always be static, return default mass properties
        return .{};
    }

    // See Shape::GetMaterial
    pub fn getMaterial(self: *const HeightFieldShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        // Decode ID
        const d = self.decodeSubShapeID(sub_shape_id);

        // Fetch the material
        return self.getMaterialAt(d.x, d.y);
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const HeightFieldShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = local_surface_position;

        // Decode ID
        const d = self.decodeSubShapeID(sub_shape_id);
        const x = d.x;
        const y = d.y;

        // Fetch vertices that both triangles share
        const x1y1 = self.getPosition(x, y);
        const x2y2 = self.getPosition(x + 1, y + 1);

        // Get normal depending on which triangle was selected
        var normal: Vec3 = undefined;
        if (d.triangle == 0) {
            const x1y2 = self.getPosition(x, y + 1);
            normal = x2y2.sub(x1y2).cross(x1y1.sub(x1y2));
        } else {
            const x2y1 = self.getPosition(x + 1, y);
            normal = x1y1.sub(x2y1).cross(x2y2.sub(x2y1));
        }

        return normal.normalized();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const HeightFieldShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        _ = direction;

        // Decode ID
        const d = self.decodeSubShapeID(sub_shape_id);
        const x = d.x;
        const y = d.y;

        // Fetch the triangle
        out_vertices.resize(3);
        const vertices = out_vertices.slice();
        vertices[0] = self.getPosition(x, y);
        const v2 = self.getPosition(x + 1, y + 1);
        if (d.triangle == 0) {
            vertices[1] = self.getPosition(x, y + 1);
            vertices[2] = v2;
        } else {
            vertices[1] = v2;
            vertices[2] = self.getPosition(x + 1, y);
        }

        // Flip triangle if scaled inside out
        if (ScaleHelpers.isInsideOut(scale))
            std.mem.swap(Vec3, &vertices[1], &vertices[2]);

        // Transform to world space
        const transform = center_of_mass_transform.preScaled(scale);
        for (vertices) |*v|
            v.* = transform.mulVec3(v.*);
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const HeightFieldShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        _ = .{ self, center_of_mass_transform, scale, surface };
        if (Core.enable_asserts) @panic("Not supported");

        // Jolt leaves the out parameters untouched
        return .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    /// Visit the entire height field using a visitor pattern
    /// Note: Used to be inlined but this triggers a bug in MSVC where it will not free the memory allocated by alloca which causes a stack overflow when WalkHeightField is called in a loop (clang does it correct)
    fn walkHeightField(self: *const HeightFieldShape, visitor: anytype) void {
        var ctx = DecodingContext.init(self);
        ctx.walkHeightField(visitor);
    }

    // See Shape::CastRay
    pub fn castRay(self: *const HeightFieldShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const Visitor = struct {
            hit: *RayCastResult,
            ray_origin: Vec3,
            ray_direction: Vec3,
            ray_inv_direction: RayInvDirection,
            shape: *const HeightFieldShape,
            sub_shape_id_creator: SubShapeIDCreator,
            return_value: bool = false,
            distance_stack: [stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.hit.fraction <= 0.0;
            }

            pub fn shouldVisitRangeBlock(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.hit.fraction;
            }

            pub fn visitRangeBlock(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Test bounds of 4 children
                const distance = rayAABox4(v.ray_origin, v.ray_inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.hit.fraction, properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
                const fraction = rayTriangle(v.ray_origin, v.ray_direction, v0, v1, v2);
                if (fraction < v.hit.fraction) {
                    // It's a closer hit
                    v.hit.fraction = fraction;
                    v.hit.sub_shape_id2 = v.shape.encodeSubShapeID(v.sub_shape_id_creator, x, y, triangle);
                    v.return_value = true;
                }
            }
        };

        var visitor: Visitor = .{
            .hit = hit,
            .ray_origin = ray.origin,
            .ray_direction = ray.direction,
            .ray_inv_direction = .init(ray.direction),
            .shape = self,
            .sub_shape_id_creator = sub_shape_id_creator,
        };
        self.walkHeightField(&visitor);

        return visitor.return_value;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const HeightFieldShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const Visitor = struct {
            collector: *CastRayCollector,
            ray_origin: Vec3,
            ray_direction: Vec3,
            ray_inv_direction: RayInvDirection,
            back_face_mode: BackFaceMode,
            shape: *const HeightFieldShape,
            sub_shape_id_creator: SubShapeIDCreator,
            distance_stack: [stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.collector.shouldEarlyOut();
            }

            pub fn shouldVisitRangeBlock(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.collector.getEarlyOutFraction();
            }

            pub fn visitRangeBlock(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Test bounds of 4 children
                const distance = rayAABox4(v.ray_origin, v.ray_inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.collector.getEarlyOutFraction(), properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *const @This(), x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
                // Back facing check
                if (v.back_face_mode == .ignore_back_faces and v2.sub(v0).cross(v1.sub(v0)).dot(v.ray_direction) < 0)
                    return;

                // Check the triangle
                const fraction = rayTriangle(v.ray_origin, v.ray_direction, v0, v1, v2);
                if (fraction < v.collector.getEarlyOutFraction()) {
                    var hit: RayCastResult = .{};
                    hit.body_id = TransformedShape.getBodyID(v.collector.getContext());
                    hit.fraction = fraction;
                    hit.sub_shape_id2 = v.shape.encodeSubShapeID(v.sub_shape_id_creator, x, y, triangle);
                    v.collector.addHit(&hit);
                }
            }
        };

        var visitor: Visitor = .{
            .collector = collector,
            .ray_origin = ray.origin,
            .ray_direction = ray.direction,
            .ray_inv_direction = .init(ray.direction),
            .back_face_mode = ray_cast_settings.back_face_mode_triangles,
            .shape = self,
            .sub_shape_id_creator = sub_shape_id_creator,
        };
        self.walkHeightField(&visitor);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const HeightFieldShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // A height field doesn't have volume, so we can't test insideness
        _ = .{ self, point, sub_shape_id_creator, collector, shape_filter };
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const HeightFieldShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        const Visitor = struct {
            base: CollideSoftBodyVerticesVsTriangles,
            distance_stack: [stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                _ = v;
                return false;
            }

            pub fn shouldVisitRangeBlock(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.base.closest_distance_sq;
            }

            pub fn visitRangeBlock(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Scale the bounding boxes of this node
                const b = AABox4.aabox4Scale(v.base.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Get distance to vertex
                var dist_sq = AABox4.aabox4DistanceSqToPointVec3(v.base.local_position, b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z);

                // Clear distance for invalid bounds
                dist_sq = Vec4.select(Vec4.replicate(math.flt_max), dist_sq, Vec4.lessOrEqual(bounds_min_y, bounds_max_y));

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(dist_sq, v.base.closest_distance_sq, properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
                _ = .{ x, y, triangle };
                v.base.processTriangle(v0, v1, v2);
            }
        };

        var visitor: Visitor = .{ .base = .init(center_of_mass_transform, scale) };

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                visitor.base.startVertex(&v);
                self.walkHeightField(&visitor);
                visitor.base.finishVertex(&v, colliding_shape_index);
            }
        }
    }

    /// CollisionDispatch function: cast a convex shape against a height field (sCastConvexVsHeightField)
    pub fn castConvexVsHeightField(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        const Visitor = struct {
            base: CastConvexVsTriangles,
            shape2: *const HeightFieldShape = undefined,
            inv_direction: RayInvDirection = undefined,
            box_center: Vec3 = undefined,
            box_extent: Vec3 = undefined,
            sub_shape_id_creator2: SubShapeIDCreator = undefined,
            distance_stack: [stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitRangeBlock(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.base.collector.getPositiveEarlyOutFraction();
            }

            pub fn visitRangeBlock(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Scale the bounding boxes of this node
                var b = AABox4.aabox4Scale(v.base.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Enlarge them by the casted shape's box extents
                AABox4.aabox4EnlargeWithExtent(v.box_extent, &b.min_x, &b.min_y, &b.min_z, &b.max_x, &b.max_y, &b.max_z);

                // Test bounds of 4 children
                var distance = rayAABox4(v.box_center, v.inv_direction, b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z);

                // Clear distance for invalid bounds
                distance = Vec4.select(Vec4.replicate(math.flt_max), distance, Vec4.lessOrEqual(bounds_min_y, bounds_max_y));

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.base.collector.getPositiveEarlyOutFraction(), properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
                // Create sub shape id for this part
                const triangle_sub_shape_id = v.shape2.encodeSubShapeID(v.sub_shape_id_creator2, x, y, triangle);

                // Determine active edges
                const active_edges = v.shape2.getEdgeFlags(x, y, triangle);

                v.base.cast(v0, v1, v2, active_edges, triangle_sub_shape_id);
            }
        };

        std.debug.assert(shape.getSubType() == .height_field);
        const height_field = shape.cast(HeightFieldShape);

        var visitor: Visitor = .{ .base = .init(shape_cast, shape_cast_settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector) };
        visitor.shape2 = height_field;
        visitor.inv_direction.set(shape_cast.direction);
        visitor.box_center = shape_cast.shape_world_bounds.getCenter();
        visitor.box_extent = shape_cast.shape_world_bounds.getExtent().add(Vec3.replicate(shape_cast_settings.extra_convex_radius));
        visitor.sub_shape_id_creator2 = sub_shape_id_creator2;
        height_field.walkHeightField(&visitor);
    }

    /// CollisionDispatch function: cast a sphere against a height field (sCastSphereVsHeightField)
    pub fn castSphereVsHeightField(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        const Visitor = struct {
            base: CastSphereVsTriangles,
            shape2: *const HeightFieldShape = undefined,
            inv_direction: RayInvDirection = undefined,
            sub_shape_id_creator2: SubShapeIDCreator = undefined,
            distance_stack: [stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitRangeBlock(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.base.collector.getPositiveEarlyOutFraction();
            }

            pub fn visitRangeBlock(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Scale the bounding boxes of this node
                var b = AABox4.aabox4Scale(v.base.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Enlarge them by the radius of the sphere
                AABox4.aabox4EnlargeWithExtent(Vec3.replicate(v.base.radius), &b.min_x, &b.min_y, &b.min_z, &b.max_x, &b.max_y, &b.max_z);

                // Test bounds of 4 children
                var distance = rayAABox4(v.base.start, v.inv_direction, b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z);

                // Clear distance for invalid bounds
                distance = Vec4.select(Vec4.replicate(math.flt_max), distance, Vec4.lessOrEqual(bounds_min_y, bounds_max_y));

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.base.collector.getPositiveEarlyOutFraction(), properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
                // Create sub shape id for this part
                const triangle_sub_shape_id = v.shape2.encodeSubShapeID(v.sub_shape_id_creator2, x, y, triangle);

                // Determine active edges
                const active_edges = v.shape2.getEdgeFlags(x, y, triangle);

                v.base.cast(v0, v1, v2, active_edges, triangle_sub_shape_id);
            }
        };

        std.debug.assert(shape.getSubType() == .height_field);
        const height_field = shape.cast(HeightFieldShape);

        var visitor: Visitor = .{ .base = .init(shape_cast, shape_cast_settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector) };
        visitor.shape2 = height_field;
        visitor.inv_direction.set(shape_cast.direction);
        visitor.sub_shape_id_creator2 = sub_shape_id_creator2;
        height_field.walkHeightField(&visitor);
    }

    // See Shape::GetTrianglesStart: placement new of the context (no pointers into itself: construct by value)
    pub fn getTrianglesStart(self: *const HeightFieldShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        context.emplace(HSGetTrianglesContext).* = .init(self, box, position_com, rotation, scale);
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const HeightFieldShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        comptime std.debug.assert(Shape.get_triangles_min_triangles_requested >= 1); // get_triangles_min_triangles_requested is too small
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        // Check if we're done
        const ctx = context.get(HSGetTrianglesContext);
        if (ctx.decode_ctx.isDoneWalking())
            return 0;

        // Store parameters on context
        ctx.max_triangles_requested = max_triangles_requested;
        ctx.triangle_vertices = out_triangle_vertices;
        ctx.materials = out_materials;
        ctx.should_abort = false; // Reset the abort flag
        ctx.num_triangles_found = 0;

        // Continue (or start) walking the height field
        ctx.decode_ctx.walkHeightField(ctx);
        return ctx.num_triangles_found;
    }

    /// CollisionDispatch function: collide a convex shape with a height field (sCollideConvexVsHeightField)
    pub fn collideConvexVsHeightField(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        // Get the shapes
        std.debug.assert(shape1.getType() == .convex);
        std.debug.assert(shape2.getType() == .height_field);
        const s1 = shape1.cast(ConvexShape);
        const s2 = shape2.cast(HeightFieldShape);

        const Visitor = struct {
            base: CollideConvexVsTriangles,
            shape2: *const HeightFieldShape,
            sub_shape_id_creator2: SubShapeIDCreator,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitRangeBlock(v: *const @This(), stack_top: i32) bool {
                _ = .{ v, stack_top };
                return true;
            }

            pub fn visitRangeBlock(v: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                _ = stack_top;

                // Scale the bounding boxes of this node
                const b = AABox4.aabox4Scale(v.base.scale2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Test which nodes collide
                var collides = AABox4.aabox4VsBox(v.base.bounds_of1_in_space_of2, b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z);

                // Filter out invalid bounding boxes
                collides = UVec4.bitAnd(collides, Vec4.lessOrEqual(bounds_min_y, bounds_max_y));

                return countAndSortTrues(collides, properties);
            }

            pub fn visitTriangle(v: *@This(), x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
                // Create ID for triangle
                const triangle_sub_shape_id = v.shape2.encodeSubShapeID(v.sub_shape_id_creator2, x, y, triangle);

                // Determine active edges
                const active_edges = v.shape2.getEdgeFlags(x, y, triangle);

                v.base.collide(v0, v1, v2, active_edges, triangle_sub_shape_id);
            }
        };

        var visitor: Visitor = .{
            .base = .init(s1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), collide_shape_settings, collector),
            .shape2 = s2,
            .sub_shape_id_creator2 = sub_shape_id_creator2,
        };
        s2.walkHeightField(&visitor);
    }

    /// CollisionDispatch function: collide a sphere with a height field (sCollideSphereVsHeightField)
    pub fn collideSphereVsHeightField(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        // Get the shapes
        std.debug.assert(shape1.getSubType() == .sphere);
        std.debug.assert(shape2.getType() == .height_field);
        const s1 = shape1.cast(SphereShape);
        const s2 = shape2.cast(HeightFieldShape);

        const Visitor = struct {
            base: CollideSphereVsTriangles,
            shape2: *const HeightFieldShape,
            sub_shape_id_creator2: SubShapeIDCreator,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitRangeBlock(v: *const @This(), stack_top: i32) bool {
                _ = .{ v, stack_top };
                return true;
            }

            pub fn visitRangeBlock(v: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                _ = stack_top;

                // Scale the bounding boxes of this node
                const b = AABox4.aabox4Scale(v.base.scale2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Test which nodes collide
                var collides = AABox4.aabox4VsSphereVec3(v.base.sphere_center_in2, v.base.radius_plus_max_separation_sq, b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z);

                // Filter out invalid bounding boxes
                collides = UVec4.bitAnd(collides, Vec4.lessOrEqual(bounds_min_y, bounds_max_y));

                return countAndSortTrues(collides, properties);
            }

            pub fn visitTriangle(v: *@This(), x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
                // Create ID for triangle
                const triangle_sub_shape_id = v.shape2.encodeSubShapeID(v.sub_shape_id_creator2, x, y, triangle);

                // Determine active edges
                const active_edges = v.shape2.getEdgeFlags(x, y, triangle);

                v.base.collide(v0, v1, v2, active_edges, triangle_sub_shape_id);
            }
        };

        var visitor: Visitor = .{
            .base = .init(s1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), collide_shape_settings, collector),
            .shape2 = s2,
            .sub_shape_id_creator2 = sub_shape_id_creator2,
        };
        s2.walkHeightField(&visitor);
    }

    // See Shape::SaveBinaryState
    pub fn saveBinaryState(self: *const HeightFieldShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.offset);
        stream.write(self.scale);
        stream.write(self.sample_count);
        stream.write(self.block_size);
        stream.write(self.bits_per_sample);
        stream.write(self.min_sample);
        stream.write(self.max_sample);
        stream.writeArray(u8, self.material_indices.items);
        stream.write(self.num_bits_per_material_index);

        if (self.buffer) |data| {
            stream.write(true);
            stream.writeBytes(data);
        } else {
            stream.write(false);
        }
    }

    // See Shape::SaveMaterialState (outMaterials = mMaterials)
    pub fn saveMaterialState(self: *const HeightFieldShape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
        try assignMaterials(out_materials, allocator, self.materials.items);
    }

    // See Shape::RestoreMaterialState (mMaterials.assign(inMaterials, inMaterials + inNumMaterials))
    pub fn restoreMaterialState(self: *HeightFieldShape, materials: []const PhysicsMaterialRefC) Allocator.Error!void {
        try assignMaterials(&self.materials, self.base.allocator, materials);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const HeightFieldShape) Shape.Stats {
        return .init(@sizeOf(HeightFieldShape) +
            self.materials.items.len * @sizeOf(PhysicsMaterialRefC) +
            @as(usize, self.range_blocks_size) * @sizeOf(RangeBlock) +
            @as(usize, self.height_samples_size) * @sizeOf(u8) +
            @as(usize, self.active_edges_size) * @sizeOf(u8) +
            self.material_indices.items.len * @sizeOf(u8), if (self.height_samples_size == 0) 0 else (self.sample_count - 1) * (self.sample_count - 1) * 2);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const HeightFieldShape) f32 {
        _ = self;
        return 0;
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *HeightFieldShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.offset);
        stream.read(&self.scale);
        stream.read(&self.sample_count);
        stream.read(&self.block_size);
        stream.read(&self.bits_per_sample);
        stream.read(&self.min_sample);
        stream.read(&self.max_sample);
        try stream.readArray(u8, self.base.allocator, &self.material_indices);
        stream.read(&self.num_bits_per_material_index);

        // We don't have the exact number of reserved materials anymore, but ensure that our array is big enough
        // TODO: Next time when we bump the binary serialization format of this class we should store the capacity and allocate the right amount, for now we accept a little bit of waste
        try self.materials.ensureTotalCapacity(self.base.allocator, std.math.shl(usize, 1, self.num_bits_per_material_index));

        self.cacheValues();

        var has_heights: u8 = 0; // A bool in Jolt, read as a byte so that a corrupt stream cannot produce an invalid bool
        stream.read(&has_heights);
        if (has_heights != 0) {
            try self.allocateBuffers();
            stream.readBytes(self.buffer.?);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.height_field);
        f.construct = ShapeFunctions.constructor(HeightFieldShape);
        f.color = Color.purple;

        for (ShapeFile.convex_sub_shape_types) |s| {
            r.registerCollideShape(s, .height_field, collideConvexVsHeightField);
            r.registerCastShape(s, .height_field, castConvexVsHeightField);

            r.registerCastShape(.height_field, s, CollisionDispatch.reversedCastShape);
            r.registerCollideShape(.height_field, s, CollisionDispatch.reversedCollideShape);
        }

        // Specialized collision functions
        r.registerCollideShape(.sphere, .height_field, collideSphereVsHeightField);
        r.registerCastShape(.sphere, .height_field, castSphereVsHeightField);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Walking the height field

    /// Context class for walking through all nodes of a heightfield
    const DecodingContext = struct {
        shape: *const HeightFieldShape,
        top: i32 = 0,
        properties_stack: [stack_size]u32 = undefined,

        fn init(shape: *const HeightFieldShape) DecodingContext {
            var ctx: DecodingContext = .{ .shape = shape };

            // Construct root stack entry
            ctx.properties_stack[0] = 0; // level: 0, x: 0, y: 0
            return ctx;
        }

        /// The visitor (a pointer) declares `shouldAbort()`, `shouldVisitRangeBlock(stack_top)`,
        /// `visitRangeBlock(min_x, min_y, min_z, max_x, max_y, max_z, properties, stack_top) i32` and
        /// `visitTriangle(x, y, triangle, v0, v1, v2)`
        fn walkHeightField(self: *DecodingContext, visitor: anytype) void {
            const shape = self.shape;

            // Early out if there's no collision
            if (shape.height_samples_size == 0)
                return;

            // Assert that an inside-out bounding box does not collide
            if (Core.enable_asserts) {
                var dummy = UVec4.replicate(0);
                std.debug.assert(visitor.visitRangeBlock(Vec4.replicate(-1.0e6), Vec4.replicate(1.0e6), Vec4.replicate(-1.0e6), Vec4.replicate(1.0e6), Vec4.replicate(-1.0e6), Vec4.replicate(1.0e6), &dummy, 0) == 0);
            }

            // Precalculate values relating to sample count
            const sample_count = shape.sample_count;
            const sample_count_min_1 = UVec4.replicate(sample_count - 1);

            // Precalculate values relating to block size
            const block_size = shape.block_size;
            const block_size_plus_1 = block_size + 1;
            const num_blocks = shape.getNumBlocks();
            const num_blocks_min_1 = num_blocks - 1;
            const max_level = getMaxLevel(num_blocks);
            const max_stride = (num_blocks + 1) >> 1;

            // Precalculate range block offset and stride for GetBlockOffsetAndScale
            const range_block = getRangeBlockOffsetAndStride(num_blocks, max_level);
            const range_block_offset = range_block.offset;
            const range_block_stride = range_block.stride;

            // Allocate space for vertices and 'no collision' flags (Jolt: JPH_STACK_ALLOC of Square(block_size_plus_1) entries)
            std.debug.assert(block_size <= max_block_size);
            const max_array_size = (max_block_size + 1) * (max_block_size + 1);
            var vertices: [max_array_size]Vec3 = undefined;
            var no_collision: [max_array_size]bool = @splat(true); // Jolt leaves these uninitialized, only written entries are read

            // Splat offsets
            const ox = shape.offset.splatX();
            const oy = shape.offset.splatY();
            const oz = shape.offset.splatZ();

            // Splat scales
            const sx = shape.scale.splatX();
            const sy = shape.scale.splatY();
            const sz = shape.scale.splatZ();

            while (true) {
                // Decode properties
                const properties_top = self.properties_stack[@intCast(self.top)];
                const x = properties_top & mask_bits_xy;
                const y = (properties_top >> num_bits_xy) & mask_bits_xy;
                const level = properties_top >> level_shift;

                if (level >= max_level) {
                    // Determine actual range of samples (minus one because we eventually want to iterate over the triangles, not the samples)
                    const min_x = x * block_size;
                    var max_x = min_x + block_size;
                    const min_y = y * block_size;
                    var max_y = min_y + block_size;

                    // Decompress vertices of block at (x, y)
                    var dst: usize = 0;
                    var block = shape.getBlockOffsetAndScale(x, y, range_block_offset, range_block_stride);
                    var v_y = min_y;
                    while (v_y < max_y) : (v_y += 1) {
                        var v_x = min_x;
                        while (v_x < max_x) : (v_x += 1) {
                            const p = shape.getPositionInBlock(v_x, v_y, block.offset, block.scale);
                            vertices[dst] = p.position;
                            no_collision[dst] = p.no_collision;
                            dst += 1;
                        }

                        // Skip last column, these values come from a different block
                        dst += 1;
                    }

                    // Decompress block (x + 1, y)
                    var max_x_decrement: u32 = 0;
                    if (x < num_blocks_min_1) {
                        dst = block_size;
                        block = shape.getBlockOffsetAndScale(x + 1, y, range_block_offset, range_block_stride);
                        v_y = min_y;
                        while (v_y < max_y) : (v_y += 1) {
                            const p = shape.getPositionInBlock(max_x, v_y, block.offset, block.scale);
                            vertices[dst] = p.position;
                            no_collision[dst] = p.no_collision;
                            dst += block_size_plus_1;
                        }
                    } else max_x_decrement = 1; // We don't have a next block, one less triangle to test

                    // Decompress block (x, y + 1)
                    if (y < num_blocks_min_1) {
                        const start = block_size * block_size_plus_1;
                        dst = start;
                        block = shape.getBlockOffsetAndScale(x, y + 1, range_block_offset, range_block_stride);
                        var v_x = min_x;
                        while (v_x < max_x) : (v_x += 1) {
                            const p = shape.getPositionInBlock(v_x, max_y, block.offset, block.scale);
                            vertices[dst] = p.position;
                            no_collision[dst] = p.no_collision;
                            dst += 1;
                        }

                        // Decompress single sample of block at (x + 1, y + 1)
                        if (x < num_blocks_min_1) {
                            block = shape.getBlockOffsetAndScale(x + 1, y + 1, range_block_offset, range_block_stride);
                            const p = shape.getPositionInBlock(max_x, max_y, block.offset, block.scale);
                            vertices[dst] = p.position;
                            no_collision[dst] = p.no_collision;
                        }
                    } else max_y -= 1; // We don't have a next block, one less triangle to test

                    // Update max_x (we've been using it so we couldn't update it earlier)
                    max_x -= max_x_decrement;

                    // We're going to divide the vertices in 4 blocks to do one more runtime sub-division, calculate the ranges of those blocks
                    const Range = struct {
                        min_x: u32,
                        min_y: u32,
                        num_triangles_x: u32,
                        num_triangles_y: u32,
                    };
                    const half_block_size = block_size >> 1;
                    const block_size_x = max_x - min_x - half_block_size;
                    const block_size_y = max_y - min_y - half_block_size;
                    const ranges = [4]Range{
                        .{ .min_x = 0, .min_y = 0, .num_triangles_x = half_block_size, .num_triangles_y = half_block_size },
                        .{ .min_x = half_block_size, .min_y = 0, .num_triangles_x = block_size_x, .num_triangles_y = half_block_size },
                        .{ .min_x = 0, .min_y = half_block_size, .num_triangles_x = half_block_size, .num_triangles_y = block_size_y },
                        .{ .min_x = half_block_size, .min_y = half_block_size, .num_triangles_x = block_size_x, .num_triangles_y = block_size_y },
                    };

                    // Calculate the min and max of each of the blocks
                    var block_min: Mat44 = undefined;
                    var block_max: Mat44 = undefined;
                    for (ranges, 0..) |range, block_index| {
                        // Get the range for this block
                        const start = range.min_x + range.min_y * block_size_plus_1;
                        const size_x_plus_1 = range.num_triangles_x + 1;
                        const size_y_plus_1 = range.num_triangles_y + 1;

                        // Calculate where to start reading
                        var src: usize = start;
                        const stride = block_size_plus_1 - size_x_plus_1;

                        // Start range with a very large inside-out box
                        var value_min = Vec3.replicate(math.large_float);
                        var value_max = Vec3.replicate(-math.large_float);

                        // Loop over the samples to determine the min and max of this block
                        for (0..size_y_plus_1) |_| {
                            for (0..size_x_plus_1) |_| {
                                if (!no_collision[src]) {
                                    value_min = Vec3.min(value_min, vertices[src]);
                                    value_max = Vec3.max(value_max, vertices[src]);
                                }
                                src += 1;
                            }
                            src += stride;
                        }
                        block_min.setColumn4(@intCast(block_index), Vec4.fromVec3(value_min));
                        block_max.setColumn4(@intCast(block_index), Vec4.fromVec3(value_max));
                    }

                    // Transpose so we have the mins and maxes of each of the blocks in rows instead of columns
                    const transposed_min = block_min.transposed();
                    const transposed_max = block_max.transposed();

                    // Check which blocks collide
                    // Note: At this point we don't use our own stack but we do allow the visitor to use its own stack
                    // to store collision distances so that we can still early out when no closer hits have been found.
                    var colliding_blocks = UVec4.init(0, 1, 2, 3);
                    const num_results = visitor.visitRangeBlock(transposed_min.getColumn4(0), transposed_min.getColumn4(1), transposed_min.getColumn4(2), transposed_max.getColumn4(0), transposed_max.getColumn4(1), transposed_max.getColumn4(2), &colliding_blocks, self.top);

                    // Loop through the results backwards (closest first)
                    var result = num_results - 1;
                    while (result >= 0) {
                        // Calculate the min and max of this block
                        const block_index = colliding_blocks.getComponent(@intCast(result));
                        const range = ranges[block_index];
                        const block_min_x = min_x + range.min_x;
                        const block_max_x = block_min_x + range.num_triangles_x;
                        const block_min_y = min_y + range.min_y;
                        const block_max_y = block_min_y + range.num_triangles_y;

                        // Loop triangles
                        v_y = block_min_y;
                        while (v_y < block_max_y) : (v_y += 1) {
                            var v_x = block_min_x;
                            while (v_x < block_max_x) : (v_x += 1) {
                                // Get first vertex
                                const offset: usize = (v_y - min_y) * block_size_plus_1 + (v_x - min_x);
                                const start_vertex = vertices[offset..];
                                const start_no_collision = no_collision[offset..];

                                // Check if vertices shared by both triangles have collision
                                if (!start_no_collision[0] and !start_no_collision[block_size_plus_1 + 1]) {
                                    // Loop 2 triangles
                                    for (0..2) |t| {
                                        // Determine triangle vertices
                                        var v0: Vec3 = undefined;
                                        var v1: Vec3 = undefined;
                                        var v2: Vec3 = undefined;
                                        if (t == 0) {
                                            // Check third vertex
                                            if (start_no_collision[block_size_plus_1])
                                                continue;

                                            // Get vertices for triangle
                                            v0 = start_vertex[0];
                                            v1 = start_vertex[block_size_plus_1];
                                            v2 = start_vertex[block_size_plus_1 + 1];
                                        } else {
                                            // Check third vertex
                                            if (start_no_collision[1])
                                                continue;

                                            // Get vertices for triangle
                                            v0 = start_vertex[0];
                                            v1 = start_vertex[block_size_plus_1 + 1];
                                            v2 = start_vertex[1];
                                        }

                                        // Call visitor
                                        visitor.visitTriangle(v_x, v_y, @intCast(t), v0, v1, v2);

                                        // Check if we're done
                                        if (visitor.shouldAbort())
                                            return;
                                    }
                                }
                            }
                        }

                        // Fetch next block until we find one that the visitor wants to see
                        result -= 1;
                        while (result >= 0 and !visitor.shouldVisitRangeBlock(self.top + result))
                            result -= 1;
                    }
                } else {
                    // Visit child grid
                    const stride = @min(@as(u32, 1) << @intCast(level), max_stride); // At the most detailed level we store a non-power of 2 number of blocks
                    const offset = grid_offsets[level] + stride * y + x;

                    // Decode min/max height
                    std.debug.assert(offset < shape.range_blocks_size);
                    const block = UVec4.loadInt4Aligned(@ptrCast(&shape.range_blocks[offset]));
                    const bounds_miny = oy.add(sy.mul(block.expand4Uint16Lo().toFloat()));
                    const bounds_maxy = oy.add(sy.mul(block.expand4Uint16Hi().toFloat()));

                    // Calculate size of one cell at this grid level
                    const internal_cell_size = UVec4.replicate(block_size << @intCast(max_level - level - 1)); // subtract 1 from level because we have an internal grid of 2x2

                    // Calculate min/max x and z
                    const two_x = UVec4.replicate(2 * x); // multiply by two because we have an internal grid of 2x2
                    const bounds_minx = ox.add(sx.mul(internal_cell_size.mul(two_x.add(UVec4.init(0, 1, 0, 1))).toFloat()));
                    const bounds_maxx = ox.add(sx.mul(UVec4.min(internal_cell_size.mul(two_x.add(UVec4.init(1, 2, 1, 2))), sample_count_min_1).toFloat()));

                    const two_y = UVec4.replicate(2 * y);
                    const bounds_minz = oz.add(sz.mul(internal_cell_size.mul(two_y.add(UVec4.init(0, 0, 1, 1))).toFloat()));
                    const bounds_maxz = oz.add(sz.mul(UVec4.min(internal_cell_size.mul(two_y.add(UVec4.init(1, 1, 2, 2))), sample_count_min_1).toFloat()));

                    // Calculate properties of child blocks
                    var properties = UVec4.replicate(((level + 1) << level_shift) + (y << (num_bits_xy + 1)) + (x << 1)).add(UVec4.init(0, 1, 1 << num_bits_xy, (1 << num_bits_xy) + 1));

                    // Check which sub nodes to visit
                    const num_results = visitor.visitRangeBlock(bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz, &properties, self.top);

                    // Push them onto the stack
                    std.debug.assert(self.top + 4 < stack_size);
                    properties.storeInt4(self.properties_stack[@intCast(self.top)..][0..4]);
                    self.top += num_results;
                }

                // Check if we're done
                if (visitor.shouldAbort())
                    return;

                // Fetch next node until we find one that the visitor wants to see
                self.top -= 1;
                while (self.top >= 0 and !visitor.shouldVisitRangeBlock(self.top))
                    self.top -= 1;

                if (self.top < 0)
                    break;
            }
        }

        /// This can be used to have the visitor early out (visitor.shouldAbort() returns true) and later continue again (call walkHeightField() again)
        fn isDoneWalking(self: *const DecodingContext) bool {
            return self.top < 0;
        }
    };

    /// Context class for GetTrianglesStart/Next
    const HSGetTrianglesContext = struct {
        decode_ctx: DecodingContext,
        shape: *const HeightFieldShape,
        local_box: OrientedBox,
        height_field_scale: Vec3,
        local_to_world: Mat44,
        max_triangles_requested: u32 = 0,
        triangle_vertices: []Float3 = &.{},
        num_triangles_found: u32 = 0,
        materials: ?[]*const PhysicsMaterial = null,
        should_abort: bool = false,
        is_inside_out: bool,

        fn init(shape: *const HeightFieldShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) HSGetTrianglesContext {
            return .{
                .decode_ctx = .init(shape),
                .shape = shape,
                .local_box = .fromAABox(Mat44.inverseRotationTranslation(rotation, position_com), box),
                .height_field_scale = scale,
                .local_to_world = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale)),
                .is_inside_out = ScaleHelpers.isInsideOut(scale),
            };
        }

        pub fn shouldAbort(self: *const HSGetTrianglesContext) bool {
            return self.should_abort;
        }

        pub fn shouldVisitRangeBlock(self: *const HSGetTrianglesContext, stack_top: i32) bool {
            _ = .{ self, stack_top };
            return true;
        }

        pub fn visitRangeBlock(self: *const HSGetTrianglesContext, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
            _ = stack_top;

            // Scale the bounding boxes of this node
            const b = AABox4.aabox4Scale(self.height_field_scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

            // Test which nodes collide
            var collides = AABox4.aabox4VsOrientedBox(self.local_box, b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z, .{});

            // Filter out invalid bounding boxes
            collides = UVec4.bitAnd(collides, Vec4.lessOrEqual(bounds_min_y, bounds_max_y));

            return countAndSortTrues(collides, properties);
        }

        pub fn visitTriangle(self: *HSGetTrianglesContext, x: u32, y: u32, triangle: u32, v0: Vec3, v1: Vec3, v2: Vec3) void {
            _ = triangle;

            // When the buffer is full and we cannot process the triangles, abort the height field walk. The next time GetTrianglesNext is called we will continue here.
            if (self.num_triangles_found + 1 > self.max_triangles_requested) {
                self.should_abort = true;
                return;
            }

            // Store vertices as Float3
            const out = self.triangle_vertices[3 * self.num_triangles_found ..][0..3];
            if (self.is_inside_out) {
                // Reverse vertices
                self.local_to_world.mulVec3(v0).storeFloat3(&out[0]);
                self.local_to_world.mulVec3(v2).storeFloat3(&out[1]);
                self.local_to_world.mulVec3(v1).storeFloat3(&out[2]);
            } else {
                // Normal scale
                self.local_to_world.mulVec3(v0).storeFloat3(&out[0]);
                self.local_to_world.mulVec3(v1).storeFloat3(&out[1]);
                self.local_to_world.mulVec3(v2).storeFloat3(&out[2]);
            }

            // Decode material
            if (self.materials) |materials|
                materials[self.num_triangles_found] = self.shape.getMaterialAt(x, y);

            // Accumulate triangles found
            self.num_triangles_found += 1;
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's own height field tests are in ZoltTests/Physics/HeightFieldShapeTests.zig, the bit exact comparison
// with Jolt in ZoltParity/Physics/HeightFieldShapeParity.zig)

const testing = std.testing;
const Ref = @import("../../../Core/Reference.zig").Ref;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const BoxShape = @import("BoxShape.zig").BoxShape;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const RegisterTypes = @import("../../../RegisterTypes.zig");

/// Samples of a test height field: a slope (h = 0.25 x + 0.5 y) with holes at the samples where `hole(x, y)` is true
fn testSamples(allocator: Allocator, sample_count: u32, comptime hole: fn (usize, usize) bool) ![]f32 {
    const samples = try allocator.alloc(f32, @as(usize, sample_count) * sample_count);
    for (samples, 0..) |*h, i| {
        const x = i % sample_count;
        const y = i / sample_count;
        h.* = if (hole(x, y)) no_collision_value else 0.25 * @as(f32, @floatFromInt(x)) + 0.5 * @as(f32, @floatFromInt(y));
    }
    return samples;
}

fn noHoles(x: usize, y: usize) bool {
    _ = .{ x, y };
    return false;
}

fn someHoles(x: usize, y: usize) bool {
    return (x * 7 + y * 3) % 11 == 0;
}

fn allHoles(x: usize, y: usize) bool {
    _ = .{ x, y };
    return true;
}

fn saveState(allocator: Allocator, shape: *const Shape) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    var wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    shape.saveBinaryState(wrapper.streamOut());
    return out.toOwnedSlice();
}

fn restoreState(allocator: Allocator, bytes: []const u8) Allocator.Error!ShapeResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper = StreamWrapper.StreamInWrapper.init(&reader);
    return Shape.restoreFromBinaryState(allocator, wrapper.streamIn());
}

test "HeightFieldShape: float to int conversions follow Jolt's x86-64 build (cvttss2si)" {
    try testing.expectEqual(@as(i32, 3), truncToInt(3.9));
    try testing.expectEqual(@as(i32, -3), truncToInt(-3.9));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), truncToInt(-2147483648.0));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), truncToInt(2147483648.0));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), truncToInt(1.0e20));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), truncToInt(-1.0e20));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), truncToInt(std.math.nan(f32)));
    try testing.expectEqual(@as(u32, 7), truncToUint(7.5));
    try testing.expectEqual(@as(u32, 0xffffff00), truncToUint(4294967040.0));
    try testing.expectEqual(@as(u32, 0), truncToUint(std.math.nan(f32)));
    try testing.expectEqual(@as(u32, 0), truncToUint(1.0e30));
    try testing.expectEqual(@as(u32, 0xffffffff), truncToUint(-1.0)); // The low 32 bits of -1
}

test "HeightFieldShape: settings constructor, cached result and Jolt's error texts" {
    const allocator = testing.allocator;

    // HeightFieldShapeSettings(inSamples, inOffset, inScale, inSampleCount, inMaterialIndices, inMaterialList)
    const samples = try testSamples(allocator, 8, someHoles);
    defer allocator.free(samples);
    const material_a = try PhysicsMaterialSimple.create(allocator, "A", Color.red);
    const material_b = try PhysicsMaterialSimple.create(allocator, "B", Color.green);
    var material_a_ref = RefConst(PhysicsMaterial).init(material_a.material());
    defer material_a_ref.deinit();
    var material_b_ref = RefConst(PhysicsMaterial).init(material_b.material());
    defer material_b_ref.deinit();
    var indices: [49]u8 = undefined;
    for (&indices, 0..) |*m, i| m.* = @intCast(i % 2);
    const settings = try HeightFieldShapeSettings.create(allocator, samples, Vec3.init(1, 2, 3), Vec3.init(0.5, 2, 0.5), 8, .{ .material_indices = &indices, .materials = &.{ material_a.material(), material_b.material() } });
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    try testing.expectEqual(@as(usize, 64), settings.height_samples.items.len);
    try testing.expectEqual(@as(usize, 49), settings.material_indices.items.len);
    try testing.expectEqual(@as(u32, 2), material_b.material().getRefCount()); // Settings + local reference
    settings.asShapeSettings().user_data = 42;

    var result = try settings.createShape(allocator);
    defer result.deinit();
    var result2 = try settings.createShape(allocator); // Cached
    defer result2.deinit();
    try testing.expect(result.getPtr() == result2.getPtr());
    const shape = result.getPtr().?.cast(HeightFieldShape);
    try testing.expectEqual(@as(u64, 42), shape.asShape().getUserData());
    try testing.expectEqual(@as(u32, 8), shape.getSampleCount());
    try testing.expectEqual(@as(u32, 2), shape.getBlockSize());
    try testing.expect(shape.getMaterialAt(0, 0) == material_a.material());
    try testing.expect(shape.getMaterialAt(1, 0) == material_b.material());
    try testing.expect(shape.asShape().mustBeStatic());
    try testing.expectEqual(@as(f32, 0.0), shape.asShape().getVolume());
    try testing.expectEqual(@as(f32, 0.0), shape.asShape().getInnerRadius());
    try testing.expectEqual(@as(u32, 2 * 3 + 1), shape.asShape().getSubShapeIDBitsRecursive());
    try testing.expectEqual(@as(u32, 2 * 7 * 7), shape.asShape().getStats().num_triangles);

    // Invalid settings
    const Case = struct { sample_count: u32, block_size: u32 = 2, bits: u32 = 8, num_materials: u32 = 0, num_indices: u32 = 0, index: u8 = 0, error_text: []const u8 };
    const cases = [_]Case{
        .{ .sample_count = 8, .block_size = 0, .error_text = "HeightFieldShape: Block size must be in the range [2, 8]!" },
        .{ .sample_count = 8, .block_size = 1, .error_text = "HeightFieldShape: Block size must be in the range [2, 8]!" },
        .{ .sample_count = 18, .block_size = 9, .error_text = "HeightFieldShape: Block size must be in the range [2, 8]!" },
        .{ .sample_count = 8, .bits = 0, .error_text = "HeightFieldShape: Bits per sample must be in the range [1, 16]!" },
        .{ .sample_count = 8, .bits = 17, .error_text = "HeightFieldShape: Bits per sample must be in the range [1, 16]!" },
        .{ .sample_count = 8, .bits = 300, .error_text = "HeightFieldShape: Bits per sample must be in the range [1, 16]!" },
        .{ .sample_count = 2, .error_text = "HeightFieldShape: Sample count too low!" },
        .{ .sample_count = 4, .block_size = 4, .error_text = "HeightFieldShape: Sample count too low!" },
        .{ .sample_count = 0, .error_text = "HeightFieldShape: Sample count too low!" },
        .{ .sample_count = 2 * 16385, .error_text = "HeightFieldShape: Sample count too high!" },
        .{ .sample_count = 40000, .block_size = 4, .error_text = "HeightFieldShape: Size exceeds the amount of available sub shape ID bits!" },
        .{ .sample_count = 8, .num_materials = 257, .error_text = "Supporting max 256 materials per height field" },
        .{ .sample_count = 8, .num_materials = 3, .num_indices = 49, .index = 5, .error_text = "Material 5 is beyond material list (size: 3)" },
        .{ .sample_count = 8, .num_indices = 49, .error_text = "No materials present, mMaterialIndices should be empty" },
    };
    for (cases) |c| {
        var s = HeightFieldShapeSettings.initDefault(allocator);
        defer s.deinit();
        s.sample_count = c.sample_count;
        s.block_size = c.block_size;
        s.bits_per_sample = c.bits;
        if (c.sample_count <= 64) try s.height_samples.appendNTimes(allocator, 1.0, @as(usize, c.sample_count) * c.sample_count); // The checks happen before the samples are read
        for (0..c.num_materials) |_| try s.materials.append(allocator, .init(material_a.material()));
        try s.material_indices.appendNTimes(allocator, c.index, c.num_indices);
        var r = try s.asShapeSettings().createShape(allocator);
        defer r.deinit();
        try testing.expectEqualStrings(c.error_text, r.getError());
    }
}

test "HeightFieldShape: a height field without collision" {
    const allocator = testing.allocator;
    const samples = try testSamples(allocator, 8, allHoles);
    defer allocator.free(samples);
    const material = try PhysicsMaterialSimple.create(allocator, "A", Color.red);
    var settings = try HeightFieldShapeSettings.init(allocator, samples, Vec3.init(1, 2, 3), Vec3.init(2, 1, 4), 8, .{});
    defer settings.deinit();
    try settings.materials.append(allocator, .init(material.material()));
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.cast(HeightFieldShape);

    // Everything is empty, the materials are cleared
    try testing.expect(shape.buffer == null);
    try testing.expectEqual(@as(usize, 0), shape.getMaterialList().len);
    try testing.expect(shape.getMaterialAt(0, 0) == PhysicsMaterial.default);
    try testing.expectEqual(@as(usize, @sizeOf(HeightFieldShape)), shape.asShape().getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.asShape().getStats().num_triangles);
    const center = Vec3.init(1 + 0.5 * 2 * 7, 2, 3 + 0.5 * 4 * 7);
    try testing.expect(shape.asShape().getLocalBounds().eql(.init(center, center)));
    try testing.expect(shape.isNoCollision(3, 4));
    try testing.expect(shape.getPosition(3, 4).eql(Vec3.init(1 + 6, 2, 3 + 16)));
    try testing.expect(shape.projectOntoSurface(Vec3.init(5, 0, 5)) == null);
    var heights: [16]f32 = undefined;
    shape.getHeights(0, 0, 4, 4, &heights, 4);
    for (heights) |h| try testing.expectEqual(@as(f32, 2.0), h);
    var hit: RayCastResult = .{};
    try testing.expect(!shape.asShape().castRay(.init(Vec3.init(5, 10, 5), Vec3.init(0, -20, 0)), .{}, &hit));

    // Clone and binary state of a shape without buffers
    var clone = Ref(Shape).init((try shape.clone(allocator)).asShapeMut());
    defer clone.deinit();
    try testing.expect(clone.get().?.cast(HeightFieldShape).buffer == null);
    const bytes = try saveState(allocator, shape.asShape());
    defer allocator.free(bytes);
    var restored = try restoreState(allocator, bytes);
    defer restored.deinit();
    const restored_bytes = try saveState(allocator, restored.getPtr().?);
    defer allocator.free(restored_bytes);
    try testing.expectEqualSlices(u8, bytes, restored_bytes);
}

test "HeightFieldShape: positions, heights, projection and the sub shape functions" {
    const allocator = testing.allocator;
    const samples = try testSamples(allocator, 16, someHoles);
    defer allocator.free(samples);
    var settings = try HeightFieldShapeSettings.init(allocator, samples, Vec3.init(1, 2, 3), Vec3.init(1, 1, 1), 16, .{});
    defer settings.deinit();
    settings.block_size = 4;
    settings.bits_per_sample = 16;
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.cast(HeightFieldShape);

    // Positions are close to the samples and the holes are kept
    for (0..16) |y| for (0..16) |x| {
        const h = samples[y * 16 + x];
        try testing.expectEqual(h == no_collision_value, shape.isNoCollision(@intCast(x), @intCast(y)));
        if (h != no_collision_value)
            try testing.expect(shape.getPosition(@intCast(x), @intCast(y)).isClose(Vec3.init(@as(f32, @floatFromInt(x)) + 1, h + 2, @as(f32, @floatFromInt(y)) + 3), .{ .max_dist_sq = 1.0e-6 }));
    };

    // GetHeights with a negative stride (upside down)
    var heights: [8 * 4]f32 = undefined;
    shape.getHeights(4, 8, 8, 4, @as([*]f32, &heights) + 3 * 8, -8);
    for (0..4) |y| for (0..8) |x| {
        const expected = samples[(8 + y) * 16 + 4 + x];
        const value = heights[(3 - y) * 8 + x];
        if (expected == no_collision_value)
            try testing.expectEqual(no_collision_value, value)
        else
            try testing.expectApproxEqAbs(expected + 2, value, 1.0e-3);
    };

    // Project onto the surface: the lower left triangle of quad (2, 5)
    const projection = shape.projectOntoSurface(Vec3.init(3.25, 100, 8.75)).?;
    try testing.expectApproxEqAbs(@as(f32, 0.25 * 2.25 + 0.5 * 5.75 + 2), projection.position.getY(), 1.0e-3);
    const c = shape.getSubShapeCoordinates(projection.sub_shape_id);
    try testing.expectEqual(HeightFieldShape.SubShapeCoordinates{ .x = 2, .y = 5, .triangle_index = 0 }, c);

    // Surface normal of the slope, supporting face of the triangle
    const normal = shape.asShape().getSurfaceNormal(projection.sub_shape_id, projection.position);
    try testing.expect(normal.isClose(Vec3.init(-0.25, 1, -0.5).normalized(), .{ .max_dist_sq = 1.0e-8 }));
    var face: Shape.SupportingFace = .empty;
    shape.asShape().getSupportingFace(projection.sub_shape_id, Vec3.axisY(), Vec3.init(1, 1, -1), Mat44.translation(Vec3.init(10, 0, 0)), &face);
    try testing.expectEqual(@as(u32, 3), face.len);
    try testing.expect(face.get(0).isClose(shape.getPosition(2, 5).mul(Vec3.init(1, 1, -1)).add(Vec3.init(10, 0, 0)), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(face.get(2).isClose(shape.getPosition(2, 6).mul(Vec3.init(1, 1, -1)).add(Vec3.init(10, 0, 0)), .{ .max_dist_sq = 1.0e-10 })); // Flipped: inside out

    // A ray straight down hits the same triangle
    var hit: RayCastResult = .{};
    try testing.expect(shape.asShape().castRay(.init(Vec3.init(3.25, 100, 8.75), Vec3.init(0, -200, 0)), .{}, &hit));
    try testing.expect(hit.sub_shape_id2.eql(projection.sub_shape_id));
    try testing.expectApproxEqAbs(projection.position.getY(), 100 - 200 * hit.fraction, 1.0e-3);
}

test "HeightFieldShape: GetTrianglesStart / Next returns every triangle and continues after a full buffer" {
    const allocator = testing.allocator;
    const samples = try testSamples(allocator, 16, noHoles);
    defer allocator.free(samples);
    var settings = try HeightFieldShapeSettings.init(allocator, samples, Vec3.zero(), Vec3.one(), 16, .{});
    defer settings.deinit();
    settings.block_size = 4; // A block has 4 * 4 * 2 = 32 triangles
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?;

    // One call with a big buffer returns every triangle once
    var context: Shape.GetTrianglesContext = .{};
    shape.getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
    const vertices = try allocator.alloc(Float3, 3 * 1000);
    defer allocator.free(vertices);
    const materials = try allocator.alloc(*const PhysicsMaterial, 1000);
    defer allocator.free(materials);
    try testing.expectEqual(@as(u32, 2 * 15 * 15), shape.getTrianglesNext(&context, 1000, vertices, materials));
    try testing.expect(materials[0] == PhysicsMaterial.default);
    try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&context, 1000, vertices, null));

    // With a small buffer the walk is continued: a block that did not fit is visited again (Jolt's behavior), so every
    // triangle is returned at least once
    shape.getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
    var seen = [_]bool{false} ** (16 * 16 * 2);
    var total: u32 = 0;
    while (true) {
        const count = shape.getTrianglesNext(&context, 32, vertices[0 .. 3 * 32], null);
        if (count == 0) break;
        try testing.expect(count <= 32);
        total += count;
        for (0..count) |t| {
            const v0 = Vec3.fromFloat3(vertices[3 * t]);
            const v2 = Vec3.fromFloat3(vertices[3 * t + 2]);
            const x: usize = @intFromFloat(v0.getX());
            const y: usize = @intFromFloat(v0.getZ());
            const triangle: usize = if (v2.getZ() > v0.getZ()) 0 else 1; // Triangle 0 has its third vertex at (x + 1, y + 1), triangle 1 at (x + 1, y)
            seen[(y * 16 + x) * 2 + triangle] = true;
        }
    }
    try testing.expect(total >= 2 * 15 * 15);
    for (0..15) |y| for (0..15) |x| {
        try testing.expect(seen[(y * 16 + x) * 2] and seen[(y * 16 + x) * 2 + 1]);
    };

    // Jolt's limitation: when one block has more triangles than requested (block size 8: 128 triangles), every call
    // returns the first triangles of that block again and the walk never finishes
    settings.asShapeSettings().clearCachedResult();
    settings.block_size = 8;
    var result8 = try settings.asShapeSettings().createShape(allocator);
    defer result8.deinit();
    const shape8 = result8.getPtr().?;
    shape8.getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
    try testing.expectEqual(@as(u32, 32), shape8.getTrianglesNext(&context, 32, vertices[0 .. 3 * 32], null));
    const first = try allocator.dupe(Float3, vertices[0 .. 3 * 32]);
    defer allocator.free(first);
    try testing.expectEqual(@as(u32, 32), shape8.getTrianglesNext(&context, 32, vertices[0 .. 3 * 32], null));
    try testing.expectEqualSlices(Float3, first, vertices[0 .. 3 * 32]);
}

test "HeightFieldShape: collide and cast through CollisionDispatch, CollideSoftBodyVertices, registration" {
    const allocator = testing.allocator;
    const samples = try allocator.alloc(f32, 8 * 8);
    defer allocator.free(samples);
    @memset(samples, 1.0);
    var settings = try HeightFieldShapeSettings.init(allocator, samples, Vec3.zero(), Vec3.one(), 8, .{});
    defer settings.deinit();
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?;

    var sphere = SphereShape.init(allocator, 0.5, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.replicate(0.5), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    // A sphere / box that penetrates the flat height field by 0.1, in both orders
    const collide_settings: CollideShapeSettings = .{};
    for ([_]*const Shape{ sphere.asShape(), box.asShape() }) |convex| {
        for (0..2) |order| {
            var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collector.deinit();
            const convex_transform = Mat44.translation(Vec3.init(3.3, 1.4, 3.6));
            if (order == 0)
                CollisionDispatch.collideShapeVsShape(convex, shape, Vec3.one(), Vec3.one(), convex_transform, Mat44.identity(), .{}, .{}, &collide_settings, &collector.base, &.{})
            else
                CollisionDispatch.collideShapeVsShape(shape, convex, Vec3.one(), Vec3.one(), Mat44.identity(), convex_transform, .{}, .{}, &collide_settings, &collector.base, &.{});
            try collector.checkError();
            try testing.expect(collector.hits.items.len > 0);
            var deepest: f32 = 0.0;
            for (collector.hits.items) |h| {
                try testing.expect(h.penetration_depth <= 0.1 + 1.0e-3);
                deepest = @max(deepest, h.penetration_depth);
            }
            try testing.expectApproxEqAbs(@as(f32, 0.1), deepest, 1.0e-3);
        }
    }

    // A sphere cast down onto the height field
    var cast_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer cast_collector.deinit();
    const shape_cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(3.3, 3.5, 3.6)), Vec3.init(0, -4, 0));
    CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &.{}, shape, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &cast_collector.base);
    try cast_collector.checkError();
    try testing.expect(cast_collector.hits.items.len > 0);
    var first_hit: f32 = 1.0;
    for (cast_collector.hits.items) |h| first_hit = @min(first_hit, h.fraction);
    try testing.expectApproxEqAbs(@as(f32, 0.5), first_hit, 1.0e-3); // Travels 2 of the 4

    // CollideSoftBodyVertices: a vertex 0.05 below the surface
    var positions = [_]Vec3{ Vec3.init(2.3, 0.95, 2.6), Vec3.init(2.25, 5, 2.5) };
    var inv_masses = [_]f32{ 1, 1 };
    var planes = [_]Plane{ Plane.init(Vec3.zero(), 0.0), Plane.init(Vec3.zero(), 0.0) };
    var penetrations = [_]f32{ -math.flt_max, -math.flt_max };
    var indices = [_]i32{ -1, -1 };
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    shape.collideSoftBodyVertices(Mat44.identity(), Vec3.one(), &vertices, 2, 3);
    try testing.expectApproxEqAbs(@as(f32, 0.05), penetrations[0], 1.0e-3);
    try testing.expectEqual(@as(i32, 3), indices[0]);
    try testing.expect(planes[0].getNormal().isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectApproxEqAbs(@as(f32, -4), penetrations[1], 1.0e-3); // Above the surface: the closest triangle, negative penetration

    // Registration
    const functions = ShapeFunctions.get(.height_field);
    try testing.expect(functions.construct != null);
    try testing.expect(functions.color.eql(Color.purple));
    try testing.expect(RegisterTypes.registry.getCollideShape(.sphere, .height_field) == &HeightFieldShape.collideSphereVsHeightField);
    try testing.expect(RegisterTypes.registry.getCastShape(.sphere, .height_field) == &HeightFieldShape.castSphereVsHeightField);
    try testing.expect(RegisterTypes.registry.getCollideShape(.box, .height_field) == &HeightFieldShape.collideConvexVsHeightField);
    try testing.expect(RegisterTypes.registry.getCastShape(.tapered_cylinder, .height_field) == &HeightFieldShape.castConvexVsHeightField);
    try testing.expect(RegisterTypes.registry.getCollideShape(.height_field, .box) == &CollisionDispatch.reversedCollideShape);
    try testing.expect(RegisterTypes.registry.getCastShape(.height_field, .sphere) == &CollisionDispatch.reversedCastShape);
}

test "HeightFieldShape: binary state, material state, clone and stats" {
    const allocator = testing.allocator;
    const samples = try testSamples(allocator, 12, someHoles);
    defer allocator.free(samples);
    const material_a = try PhysicsMaterialSimple.create(allocator, "A", Color.red);
    const material_b = try PhysicsMaterialSimple.create(allocator, "B", Color.blue);
    var indices: [11 * 11]u8 = undefined;
    for (&indices, 0..) |*m, i| m.* = @intCast(i % 3 % 2);
    var settings = try HeightFieldShapeSettings.init(allocator, samples, Vec3.init(-1, 0.5, 2), Vec3.init(1.5, 0.75, 1.25), 12, .{ .material_indices = &indices, .materials = &.{ material_a.material(), material_b.material() } });
    defer settings.deinit();
    settings.block_size = 3;
    settings.bits_per_sample = 5;
    settings.materials_capacity = 7;
    settings.asShapeSettings().user_data = 0x1234;
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.cast(HeightFieldShape);
    try testing.expectEqual(@as(u32, 3), shape.num_bits_per_material_index); // The capacity of 7 materials
    try testing.expect(shape.materials.capacity >= 7);

    // Stats
    const stats = shape.asShape().getStats();
    try testing.expectEqual(@sizeOf(HeightFieldShape) + 2 * @sizeOf(PhysicsMaterialRefC) + shape.range_blocks.len * @sizeOf(HeightFieldShape.RangeBlock) + shape.height_samples.len + shape.active_edges.len + shape.material_indices.items.len, stats.size_bytes);
    try testing.expectEqual(@as(u32, 2 * 11 * 11), stats.num_triangles);

    // Save and restore (the materials separately)
    const bytes = try saveState(allocator, shape.asShape());
    defer allocator.free(bytes);
    var restored = try restoreState(allocator, bytes);
    defer restored.deinit();
    const restored_shape = restored.getPtr().?;
    var materials: PhysicsMaterialList = .empty;
    defer {
        for (materials.items) |*m| m.deinit();
        materials.deinit(allocator);
    }
    try shape.asShape().saveMaterialState(allocator, &materials);
    try testing.expectEqual(@as(usize, 2), materials.items.len);
    try restored_shape.restoreMaterialState(materials.items);
    const restored_bytes = try saveState(allocator, restored_shape);
    defer allocator.free(restored_bytes);
    try testing.expectEqualSlices(u8, bytes, restored_bytes);
    try testing.expectEqual(@as(u64, 0x1234), restored_shape.getUserData());
    try testing.expect(restored_shape.cast(HeightFieldShape).getMaterialAt(1, 0) == material_b.material());
    try testing.expect(restored_shape.cast(HeightFieldShape).materials.capacity >= 8); // 1 << num_bits_per_material_index

    // Clone
    var clone = Ref(Shape).init((try shape.clone(allocator)).asShapeMut());
    defer clone.deinit();
    const clone_bytes = try saveState(allocator, clone.get().?);
    defer allocator.free(clone_bytes);
    try testing.expectEqualSlices(u8, bytes, clone_bytes);
    try testing.expect(clone.get().?.cast(HeightFieldShape).materials.capacity >= shape.materials.capacity);
    try testing.expectEqual(@as(u32, 5), material_a.material().getRefCount()); // Settings, shape, `materials`, the restored shape and the clone
}

test "HeightFieldShape: every allocation failure is reported" {
    const allocator = testing.allocator;
    const samples = try testSamples(allocator, 16, someHoles);
    defer allocator.free(samples);
    const material_a = try PhysicsMaterialSimple.create(allocator, "A", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material_a.material());
    defer material_ref.deinit();
    const indices: [15 * 15]u8 = @splat(0);

    // Settings constructor
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        const settings = HeightFieldShapeSettings.create(failing.allocator(), samples, Vec3.zero(), Vec3.one(), 16, .{ .material_indices = &indices, .materials = &.{material_a.material()} }) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
        settings_ref.deinit();
        break;
    }

    // createShape: out of memory is returned and not cached
    var settings = try HeightFieldShapeSettings.init(allocator, samples, Vec3.zero(), Vec3.one(), 16, .{ .material_indices = &indices, .materials = &.{material_a.material()} });
    defer settings.deinit();
    settings.materials_capacity = 3;
    settings.block_size = 4;
    fail_index = 0;
    var allocations: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var r = settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expect(settings.base.cached_result.isEmpty());
            continue;
        };
        defer r.deinit();
        try testing.expect(r.isValid());
        allocations = failing.allocations;
        settings.asShapeSettings().clearCachedResult();
        break;
    }
    try testing.expect(allocations >= 8); // The shape, materials, buffers, quantized samples, ranges, normals, material indices

    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    settings.asShapeSettings().clearCachedResult();
    const shape = result.getPtr().?.cast(HeightFieldShape);

    // Clone
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        const clone = shape.clone(failing.allocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        var clone_ref = Ref(Shape).init(clone.asShapeMut());
        clone_ref.deinit();
        break;
    }

    // Restore
    const bytes = try saveState(allocator, shape.asShape());
    defer allocator.free(bytes);
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var r = restoreState(failing.allocator(), bytes) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        defer r.deinit();
        try testing.expect(r.isValid());
        const materials = [_]PhysicsMaterialRefC{material_ref};
        r.getPtr().?.restoreMaterialState(&materials) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        break;
    }

    // SetHeights (temp allocator) and SetMaterials (temp allocator and the material list of the shape)
    var heights: [8 * 8]f32 = undefined;
    for (&heights, 0..) |*h, i| h.* = @floatFromInt(i % 5);
    const material_b = try PhysicsMaterialSimple.create(allocator, "B", Color.green);
    var material_b_ref = RefConst(PhysicsMaterial).init(material_b.material());
    defer material_b_ref.deinit();
    var patch: [4 * 4]u8 = undefined;
    for (&patch, 0..) |*m, i| m.* = @intCast(i % 2);
    const new_list = [_]PhysicsMaterialRefC{ material_ref, material_b_ref };
    fail_index = 0;
    while (true) : (fail_index += 1) {
        // A fresh shape whose own allocator fails too
        var failing = std.testing.FailingAllocator.init(allocator, .{});
        var r = try settings.asShapeSettings().createShape(failing.allocator());
        defer r.deinit();
        settings.asShapeSettings().clearCachedResult();
        const s = r.getPtr().?.castMut(HeightFieldShape);
        failing.fail_index = failing.alloc_index + fail_index;
        var temp = TempAllocatorMalloc.init(failing.allocator());
        s.setHeights(4, 4, 8, 8, &heights, 8, temp.tempAllocator(), .{}) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        const set = s.setMaterials(2, 3, 4, 4, &patch, 4, &new_list, temp.tempAllocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        try testing.expect(set);
        try testing.expect(s.getMaterialAt(3, 3) == material_b.material());
        break;
    }
    try testing.expect(fail_index >= 3); // Temp heights, normals, remap table (+ the material list and indices)
}
