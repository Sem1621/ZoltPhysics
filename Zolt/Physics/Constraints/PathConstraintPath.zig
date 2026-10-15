//! Port of: Jolt/Physics/Constraints/PathConstraintPath.h, Jolt/Physics/Constraints/PathConstraintPath.cpp
//! Status: complete
//!
//! JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(PathConstraintPath): TODO(serialization) (the
//! binary state is ported). JPH_DEBUG_RENDERER (DrawPath with its helpers sTransformPathPoint / sDrawPathSegment):
//! TODO(debug_renderer).
//!
//! Pattern A RefTarget root like GroupFilter (Docs/Zolt/CollisionArchitecture.md, D1, D2, D7 and D8): the vtable, the
//! atomic reference count, the allocator that frees the most derived object (and the memory it owns) and Jolt's data.
//! - PathConstraintPath is abstract (GetPathMaxFraction, GetClosestPoint and GetPointOnPath are pure virtual), so it
//!   keeps the bodies of its other virtual functions (SaveBinaryState, RestoreBinaryState) in `impl`. A derived path has
//!   a field `base: PathConstraintPath` built with `.base = .init(@This(), allocator)` (the protected constructor), lists
//!   its `overrides` and declares `pub const rtti_name` (JPH_RTTI). `new X` is `X.create(allocator)` (reference count
//!   0, put it in a `Ref` / `RefConst`). A path on the stack or embedded in another object calls `setEmbedded()` before
//!   references are taken and `deinit()` at the end.
//! - GetPointOnPath has four out parameters: `getPointOnPath(fraction) PointOnPath{ .position, .tangent, .normal,
//!   .binormal }` (PortingGuide section 4).
//! - The protected copy constructor (it does not copy the reference count) is `initCopy`, used by the copy functions of
//!   derived paths (`PathConstraintPathHermite.clone`).
//! - SaveBinaryState writes Jolt's RTTI hash of the class name (`rtti_name` is a vtable data entry).
//!   sRestoreFromBinaryState (StreamUtils::RestoreObject) finds the class in the comptime list
//!   `RegisterTypes.path_constraint_path_types` (Jolt's PathConstraintPathHermite plus the `path_constraint_path_types`
//!   of the `zolt_user_types` module, until the Factory of Phase 8), skipping classes that declare `is_stub`. Restoring
//!   allocates, so it returns `Allocator.Error!PathResult`; Jolt's errors ("Failed to read type hash", ...) are values in
//!   the result. A hash that is not in the list (including the hash of the abstract PathConstraintPath itself, or of a
//!   registered class that is not a path, which Jolt's Factory would create and reinterpret) gives "Failed to create
//!   instance of type".

const std = @import("std");
const Allocator = std.mem.Allocator;
const HashCombine = @import("../../Core/HashCombine.zig");
const RefCount = @import("../../Core/Reference.zig").RefCount;
const Ref = @import("../../Core/Reference.zig").Ref;
const Result = @import("../../Core/Result.zig").Result;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const virtual = @import("../../Core/Virtual.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RegisterTypes = @import("../../RegisterTypes.zig");

/// The path for a path constraint. It allows attaching two bodies to each other while giving the second body the freedom to move along a path relative to the first.
pub const PathConstraintPath = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_ABSTRACT(JPH_EXPORT, PathConstraintPath)

    pub const PathResult = Result(Ref(PathConstraintPath));

    /// The point, tangent, normal and binormal that GetPointOnPath returns (Jolt's out parameters)
    pub const PointOnPath = struct {
        /// Returns the closest position to inSearchPosition on the path.
        position: Vec3,
        /// Returns the tangent to the path at outPathPosition (the vector that follows the direction of the path)
        tangent: Vec3,
        /// Return the normal to the path at outPathPosition (a vector that's perpendicular to outPathTangent)
        normal: Vec3,
        /// Returns the binormal to the path at outPathPosition (a vector so that normal cross tangent = binormal)
        binormal: Vec3,
    };

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Class name of the most derived class (JPH_RTTI, data entry: every concrete class declares `rtti_name`)
        rtti_name: []const u8,
        /// Virtual destructor to ensure that derived types get their destructors called (generated destructor chain)
        deinit: *const fn (self: *PathConstraintPath) void,
        /// delete this (generated)
        destroy: *const fn (self: *PathConstraintPath) void,

        /// Gets the max fraction along the path. I.e. sort of the length of the path.
        getPathMaxFraction: *const fn (self: *const PathConstraintPath) f32,

        /// Get the globally closest point on the curve (Could be slow!)
        /// @param position Position to find closest point for
        /// @param fraction_hint Last known fraction along the path (can be used to speed up the search)
        /// @return Fraction of closest point along the path
        getClosestPoint: *const fn (self: *const PathConstraintPath, position: Vec3, fraction_hint: f32) f32,

        /// Given the fraction along the path, get the point, tangent and normal.
        /// @param fraction Fraction along the path [0, GetPathMaxFraction()].
        /// @return The position, tangent, normal and binormal (see PointOnPath)
        getPointOnPath: *const fn (self: *const PathConstraintPath, fraction: f32) PointOnPath,

        /// Saves the contents of the path in binary form to inStream.
        saveBinaryState: *const fn (self: *const PathConstraintPath, stream: StreamOut) void,

        /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
        restoreBinaryState: *const fn (self: *PathConstraintPath, stream: StreamIn) Allocator.Error!void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<PathConstraintPath>)
    ref_count: RefCount = .{},
    /// Frees this path (heap paths) and the memory it owns
    allocator: Allocator,

    /// If the path is looping or not. If a path is looping, the first and last point are automatically connected to each other. They should not be the same points.
    /// (private in Jolt, use isLooping / setIsLooping)
    is_looping: bool = false,

    /// Path classes that restoreFromBinaryState can create (Factory::sInstance until Phase 8). Each one declares
    /// `rtti_name` and `createDefault(allocator) Allocator.Error!*T` (its default constructor on the heap).
    pub const path_constraint_path_types = RegisterTypes.path_constraint_path_types;

    /// Constructor (protected in Jolt), called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) PathConstraintPath {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// The protected copy constructor: copies the path data of `other`, not its reference count. `vtable` is the vtable
    /// of the most derived class of the copy.
    pub fn initCopy(vtable: *const VTable, allocator: Allocator, other: *const PathConstraintPath) PathConstraintPath {
        return .{ .vtable = vtable, .allocator = allocator, .is_looping = other.is_looping };
    }

    /// The vtable of path class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<PathConstraintPath>

    /// Add a reference to this object
    pub fn addRef(self: *const PathConstraintPath) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const PathConstraintPath) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this path as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const PathConstraintPath) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const PathConstraintPath) u32 {
        return self.ref_count.get();
    }

    /// Destructor of a path that is not on the heap (embedded / stack)
    pub fn deinit(self: *PathConstraintPath) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// Gets the max fraction along the path. I.e. sort of the length of the path.
    pub fn getPathMaxFraction(self: *const PathConstraintPath) f32 {
        return self.vtable.getPathMaxFraction(self);
    }

    /// Get the globally closest point on the curve (Could be slow!), see VTable.getClosestPoint
    pub fn getClosestPoint(self: *const PathConstraintPath, position: Vec3, fraction_hint: f32) f32 {
        return self.vtable.getClosestPoint(self, position, fraction_hint);
    }

    /// Given the fraction along the path, get the point, tangent and normal, see VTable.getPointOnPath
    pub fn getPointOnPath(self: *const PathConstraintPath, fraction: f32) PointOnPath {
        return self.vtable.getPointOnPath(self, fraction);
    }

    /// If the path is looping or not. If a path is looping, the first and last point are automatically connected to each other. They should not be the same points.
    pub fn setIsLooping(self: *PathConstraintPath, looping: bool) void {
        self.is_looping = looping;
    }

    pub fn isLooping(self: *const PathConstraintPath) bool {
        return self.is_looping;
    }

    // TODO(debug_renderer): DrawPath (Draw the path relative to inBaseTransform. Used for debug purposes.)

    /// Saves the contents of the path in binary form to inStream.
    pub fn saveBinaryState(self: *const PathConstraintPath, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
    pub fn restoreBinaryState(self: *PathConstraintPath, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// Creates a Shape of the correct type and restores its contents from the binary stream inStream.
    /// (StreamUtils::RestoreObject with the comptime type list `path_constraint_path_types` instead of the Factory)
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!PathResult {
        var result: PathResult = .empty;

        // Read the hash of the type
        var hash: u32 = 0;
        stream.read(&hash);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read type hash");
            return result;
        }

        // Get the RTTI for the type and construct it (Zolt: a class that is still a stub, `is_stub`, is not known yet)
        const object: *PathConstraintPath = inline for (path_constraint_path_types) |T| {
            if (comptime @hasDecl(T, "is_stub")) continue;
            if (hash == comptime rttiHash(T.rtti_name)) break virtual.upcast(PathConstraintPath, try T.createDefault(allocator));
        } else {
            result.setError("Failed to create instance of type");
            return result;
        };

        // Read the data of the type
        var ref = Ref(PathConstraintPath).init(object);
        defer ref.deinit();
        try object.restoreBinaryState(stream);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to restore object");
            return result;
        }

        result.set(ref.clone());
        return result;
    }

    /// GetRTTI()->GetHash() of the most derived class
    pub fn getRTTIHash(self: *const PathConstraintPath) u32 {
        return rttiHash(self.vtable.rtti_name);
    }

    /// RTTI::GetHash: FNV-1a hash of the class name, folded from 64 to 32 bits
    pub fn rttiHash(name: []const u8) u32 {
        // Perform diffusion step to get from 64 to 32 bits (see https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function)
        const hash = HashCombine.hashString(name);
        return @truncate(hash ^ (hash >> 32));
    }

    /// The bodies of the virtual functions of PathConstraintPath (GetPathMaxFraction, GetClosestPoint and GetPointOnPath
    /// are pure virtual)
    pub const impl = struct {
        pub fn saveBinaryState(self: *const PathConstraintPath, stream: StreamOut) void {
            stream.write(self.getRTTIHash());
            stream.write(self.is_looping);
        }

        pub fn restoreBinaryState(self: *PathConstraintPath, stream: StreamIn) Allocator.Error!void {
            // Type hash read by sRestoreFromBinaryState
            stream.read(&self.is_looping);
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the Hermite path is tested in PathConstraintPathHermite.zig, the parity in ZoltParity/Physics/ConstraintsParity.zig)

const testing = std.testing;
const math = @import("../../Math/Math.zig");
const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

/// A user path: a straight line along X of length `length`, it keeps the base class binary state
const TestLinePath = struct {
    pub const rtti_name = "TestLinePath";
    pub const overrides = .{ .getPathMaxFraction, .getClosestPoint, .getPointOnPath };

    base: PathConstraintPath,
    length: f32 = 2.0,
    destructed: ?*u32 = null,

    pub fn destruct(self: *TestLinePath) void {
        if (self.destructed) |d| d.* += 1;
    }

    pub fn getPathMaxFraction(self: *const TestLinePath) f32 {
        return self.length;
    }

    pub fn getClosestPoint(self: *const TestLinePath, position: Vec3, fraction_hint: f32) f32 {
        _ = fraction_hint;
        return math.max(@as(f32, 0.0), math.min(self.length, position.getX()));
    }

    pub fn getPointOnPath(self: *const TestLinePath, fraction: f32) PathConstraintPath.PointOnPath {
        _ = self;
        return .{ .position = Vec3.init(fraction, 0, 0), .tangent = Vec3.axisX(), .normal = Vec3.axisY(), .binormal = Vec3.axisZ() };
    }
};

test "PathConstraintPath: virtual dispatch, looping, reference counting, the base class binary state" {
    const allocator = testing.allocator;

    // A heap path: the last release destroys it through the vtable
    var destructed: u32 = 0;
    const path = try allocator.create(TestLinePath);
    path.* = .{ .base = .init(TestLinePath, allocator), .destructed = &destructed };
    var ref = Ref(PathConstraintPath).init(&path.base);
    try testing.expectEqual(@as(u32, 1), path.base.getRefCount());
    const base: *const PathConstraintPath = &path.base;
    try testing.expectEqual(@as(f32, 2.0), base.getPathMaxFraction());
    try testing.expectEqual(@as(f32, 1.5), base.getClosestPoint(Vec3.init(1.5, 3, 4), 0.0));
    try testing.expectEqual(@as(f32, 2.0), base.getClosestPoint(Vec3.init(5, 0, 0), 0.0));
    const point = base.getPointOnPath(0.25);
    try testing.expect(point.position.eql(Vec3.init(0.25, 0, 0)));
    try testing.expect(point.tangent.eql(Vec3.axisX()) and point.normal.eql(Vec3.axisY()) and point.binormal.eql(Vec3.axisZ()));

    // Looping: not by default
    try testing.expect(!base.isLooping());
    path.base.setIsLooping(true);
    try testing.expect(base.isLooping());

    // The base class binary state: the RTTI hash and mIsLooping
    var buffer: [16]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    base.saveBinaryState(out.streamOut());
    try testing.expectEqual(@as(usize, 5), writer.buffered().len);
    try testing.expectEqual(PathConstraintPath.rttiHash("TestLinePath"), std.mem.readInt(u32, buffer[0..4], .little));
    try testing.expectEqual(PathConstraintPath.rttiHash("TestLinePath"), base.getRTTIHash());
    try testing.expectEqual(@as(u8, 1), buffer[4]);

    // The class is not in path_constraint_path_types, so it cannot be restored
    {
        var reader: std.Io.Reader = .fixed(writer.buffered());
        var in = StreamInWrapper.init(&reader);
        var restored = try PathConstraintPath.restoreFromBinaryState(allocator, in.streamIn());
        defer restored.deinit();
        try testing.expectEqualStrings("Failed to create instance of type", restored.getError());
    }

    // The base class RestoreBinaryState reads mIsLooping (the hash is read by sRestoreFromBinaryState)
    {
        var reader: std.Io.Reader = .fixed(buffer[4..5]);
        var in = StreamInWrapper.init(&reader);
        path.base.setIsLooping(false);
        try path.base.restoreBinaryState(in.streamIn());
        try testing.expect(path.base.isLooping());
    }

    // The copy constructor copies the data, not the reference count
    var copy: TestLinePath = .{ .base = .initCopy(path.base.vtable, allocator, &path.base), .length = path.length };
    copy.base.setEmbedded();
    try testing.expect(copy.base.isLooping());
    try testing.expectEqual(PathConstraintPath.rttiHash("TestLinePath"), copy.base.getRTTIHash());
    copy.base.deinit();

    ref.deinit();
    try testing.expectEqual(@as(u32, 1), destructed);
}

test "PathConstraintPath: restoreFromBinaryState errors" {
    const allocator = testing.allocator;

    // No hash
    {
        var reader: std.Io.Reader = .fixed(&.{ 1, 2 });
        var in = StreamInWrapper.init(&reader);
        var result = try PathConstraintPath.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        try testing.expectEqualStrings("Failed to read type hash", result.getError());
    }

    // The hash of the abstract base class
    {
        var bytes: [5]u8 = undefined;
        std.mem.writeInt(u32, bytes[0..4], PathConstraintPath.rttiHash("PathConstraintPath"), .little);
        bytes[4] = 0;
        var reader: std.Io.Reader = .fixed(&bytes);
        var in = StreamInWrapper.init(&reader);
        var result = try PathConstraintPath.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        try testing.expectEqualStrings("Failed to create instance of type", result.getError());
    }

    // A Hermite path that ends after the hash
    {
        var bytes: [4]u8 = undefined;
        std.mem.writeInt(u32, &bytes, PathConstraintPath.rttiHash("PathConstraintPathHermite"), .little);
        var reader: std.Io.Reader = .fixed(&bytes);
        var in = StreamInWrapper.init(&reader);
        var result = try PathConstraintPath.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        try testing.expectEqualStrings("Failed to restore object", result.getError());
    }
}
