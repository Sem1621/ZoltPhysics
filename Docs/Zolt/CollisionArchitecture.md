# Collision architecture (Phase 4)

This is the binding design for porting `Jolt/Physics/Collision`. Every decision (D1..D14) is
written as an instruction for porters. It was decided with three competing compiled prototypes
(fidelity first, Zig idiom and safety first, performance and determinism first), two judges and a
synthesis; the synthesized prototype is in commit `6fc77e5` (`ZoltTests/Prototype/`) and was removed
after the foundation port. The real files are now the reference: `Zolt/Core/Virtual.zig`,
`Zolt/Core/PlacementBuffer.zig`, `Zolt/Core/Result.zig`, `Zolt/Physics/Collision/Shape/Shape.zig`,
`ConvexShape.zig`, `SphereShape.zig` / `BoxShape.zig` (the reference shapes), `DecoratedShape.zig`,
`CompoundShape.zig`, `CompoundShapeVisitors.zig`, `CollisionDispatch.zig`, `CollisionCollectorImpl.zig`,
`TransformedShape.zig` and `Zolt/RegisterTypes.zig`. Read `PortingGuide.md` first: this document only
adds to it. Section 9 lists what the foundation port settled beyond the original design.

## 0. The four rules every porter must know

1. **Rule M (mutation).** Zig marks every `*const T` parameter `readonly` for LLVM, including
   calls through vtable function pointers. Writing through a pointer obtained from a `*const`
   with `@constCast` is undefined behavior, and ReleaseFast really drops such writes (verified:
   Debug prints 42, ReleaseFast prints 0). C++ `mutable` members and `const_cast` writes therefore
   become one of these:
   - a mutable receiver, e.g. `ShapeSettings.createShape(self: *ShapeSettings, ...)` writes the
     cache;
   - a mutable pointer passed separately, e.g. `opts.shape_filter: ?*ShapeFilter` at the query
     entry points;
   - state behind a pointer field, e.g. a filter's counter in `*u32`.

   Exactly two exceptions are allowed:
   - `RefCount` atomics (`addRef`/`release` through `*const`). LLVM never elides atomic
     read-modify-writes.
   - `release()` destroying the object after its last reference.

   The whole Phase 4 test suite also runs in ReleaseFast.
2. **Override lists.** A concrete class declares `pub const overrides = .{ .castRay, ... }`, the
   C++ `override` keywords. An abstract class keeps its bodies of virtual functions in
   `pub const impl = struct { ... }`. `virtual.make` turns every mistake into a compile error.
3. **Two error channels.** `error.OutOfMemory` is a Zig error and is never cached. Jolt's
   `Result` errors ("Invalid radius") are values in `ShapeResult` with Jolt's exact texts.
4. **No allocator in queries.** Shapes are immutable and shared after creation. Queries never
   allocate except through the collector, which owns its allocator. Creation, restore and
   `scaleShape` take an `allocator`.

## 1. Decisions

### D1 Class hierarchy, vtables, calls and casts

**Pattern A, one level per C++ class.** Each derived struct embeds its parent as the field
`base`, and the root holds `vtable`.

**Vtables.**
- A class that adds virtual functions has a `VTable` whose first field is the parent's vtable,
  which is the C++ prefix layout. Examples: `ConvexShape.VTable { base: Shape.VTable,
  getSupportFunction }` and `CompoundShape.VTable { base: Shape.VTable,
  getIntersectingSubShapes, ... }`.
- `DecoratedShape` adds no virtual functions, so it reuses `Shape.VTable`.
- `virtual.make(VTable, T)` (`Zolt/Core/Virtual.zig`) builds the table of a concrete class at
  compile time. The constructor of the introducing class calls it with the most derived type, so
  a convex shape cannot be built with a plain `Shape.VTable`:

```zig
pub const ConvexShape = struct {
    pub const shape_type: ShapeType = .convex;

    pub const VTable = struct {
        base: Shape.VTable,
        getSupportFunction: *const fn (self: *const ConvexShape, mode: SupportMode, buffer: *SupportBuffer, scale: Vec3) *const Support,
    };

    base: Shape,
    material: RefConst(PhysicsMaterial) = .empty,
    density: f32 = 1000.0,

    pub fn init(comptime T: type, allocator: Allocator, shape_sub_type: ShapeSubType, material: ?*const PhysicsMaterial) ConvexShape {
        return .{ .base = .init(&vtableFor(T).base, allocator, .convex, shape_sub_type), .material = .init(material) };
    }

    fn getVTable(self: *const ConvexShape) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    /// Returns an object that provides the GetSupport function for this shape (virtual dispatcher)
    pub fn getSupportFunction(self: *const ConvexShape, mode: SupportMode, buffer: *SupportBuffer, scale: Vec3) *const Support {
        return self.getVTable().getSupportFunction(self, mode, buffer, scale);
    }
    ...
};
```

**Entries.**
- There is one `VTable` entry per C++ virtual function, in declaration order, with Jolt's doc
  comment.
- Overloads get suffixes: `castRay` / `castRayCollector`, `shouldCollide` /
  `shouldCollidePair`.
- Out parameters become a returned struct (`getLeafShape -> LeafShape{ shape, remainder }`) or a
  `*T` parameter.
- `deinit` (destructor chain) and `destroy` (`delete this`) are generated entries.
- Data entries hold per-class constants, e.g. `rtti_name` in `PhysicsMaterial.VTable`.
- `make` builds thunks for up to 12 parameters including `self`; a longer function needs a new
  thunk in Virtual.zig.

**How `make` fills an entry.** It walks T, its parent, and so on up to the introducing class. At
each level it takes the function listed in that level's `overrides` or declared in its `impl`,
and the first match wins. Overrides of a concrete parent are therefore inherited exactly as in
C++: a user collector derived from `ClosestHitPerBodyCollisionCollector` keeps its `onBody`.
These mistakes are compile errors (messages verified in the prototype):

| Mistake | Error |
|---|---|
| pure virtual not implemented | `UserConvexShape must implement the pure virtual function Shape.getVolume` |
| override not in `overrides` | `UserConvexShape.getVolume overrides a virtual function: add .getVolume to UserConvexShape.overrides (or rename it ...)` |
| listed but private | `UserConvexShape.overrides lists getVolumeUnused, but UserConvexShape has no pub declaration getVolumeUnused` |
| listed but misspelled | `... lists getVolumeUnused, which is not a virtual function of ConvexShape.VTable` |
| wrong parameter type | `UserConvexShape.getSurfaceNormal (in the vtable of UserConvexShape): parameter 1 must have type SubShapeID` |
| `*T` receiver on a const virtual | `... the virtual function is const, the first parameter must be *const UserConvexShape` |
| top-level virtual name in an abstract class | `ConvexShape.getVolume has the name of a virtual function: an abstract class keeps its implementation in ConvexShape.impl` |

**Calls.**
- **Dispatchers.** A dispatcher (`pub fn castRay(self: *const Shape, ...)`) exists only on the
  class that introduces the virtual function. A plain-name call on a pointer to that class is
  always virtual.
- **C++ unqualified virtual call `Foo()`.** Call the dispatcher: `self.base.castRay(...)` or
  `self.asShape().castRay(...)`. A static `self.foo()` is allowed only when the C++ class is
  `final` (all shapes are; collectors are not).
- **Why abstract classes use `impl`.** An abstract class has no top-level overrides, so code
  inside `ConvexShape` can never bind statically where C++ is virtual:

```zig
pub const impl = struct {
    pub fn castRayCollector(self: *const ConvexShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        ...
        if (self.base.castRay(ray, sub_shape_id_creator, &hit)) { // Virtual call (C++ unqualified CastRay)
```
- **C++ qualified `Base::Foo()`.** For an abstract base, call `Base.impl.foo(&self.base, ...)`.
  For a concrete base, call `self.base.foo(...)`, its top-level function. If `Base` does not
  override `Foo`, name the ancestor that does. For example, `ConvexShape::IsValidScale` is
  `Shape.impl.isValidScale(self.asShape(), scale)`, and the compiler rejects a non-existent
  `ConvexShape.impl.isValidScale`.

**Types and casts.**
- `shape_type: ShapeType` and `shape_sub_type: ShapeSubType` are enums in Jolt's order.
  `all_sub_shape_types`, `convex_sub_shape_types`, `compound_sub_shape_types` and
  `decorator_sub_shape_types` are comptime arrays.
- A concrete class declares `pub const shape_sub_type`; an abstract class declares
  `pub const shape_type`.
- `shape.cast(SphereShape)` and `castMut` are the checked `static_cast` (asserted in safe
  builds). `shape.isKindOf(ConvexShape)` is the check alone.
- `virtual.upcast` / `virtual.downcast` are the unchecked forms. `asShape()` / `asShapeMut()`
  are the implicit upcasts.

**Const correctness.**
- Every query takes `*const Shape`.
- Only creation and restore take `*Shape`: `setUserData`, `restore*State`, `setMaterial`,
  `setDensity`. They run before the shape is shared.

### D2 Lifetime and ownership

**Root fields.** Every RefTarget root (Shape, ShapeSettings, PhysicsMaterial, GroupFilter) holds
`ref_count: RefCount`, `allocator: Allocator` (it frees the object and the arrays the object
owns) and, for materials, `is_static`.

**Release and destroy.** `release()` calls the generated `destroy`, which runs `deinit` (each
level's optional `destruct`, derived first, as in C++) and then
`allocator.destroy(most_derived)`:

```zig
pub fn release(self: *const Shape) void {
    if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
}

/// ~DecoratedShape
pub fn destruct(self: *DecoratedShape) void {
    self.inner_shape.deinit();
}
```

**Heap and stack objects.** `new X` becomes `X.create(allocator, ...)`, which returns the object
with refcount 0 (and `error.OutOfMemory`). Stack or member objects are embedded:

```zig
var sphere = try SphereShape.create(allocator, 1.0, .{});      // refcount 0, like `new`
var sphere_ref = RefConst(Shape).init(sphere.asShape());       // refcount 1
defer sphere_ref.deinit();                                     // last release destroys it through the vtable

var box = BoxShape.init(allocator, Vec3.one(), .{});           // on the stack
box.asShape().setEmbedded();
defer box.asShapeMut().deinit();                               // destructor chain, asserts no references are left
```

**Ownership.**
- Child shapes and materials are `RefConst(Shape)` / `RefConst(PhysicsMaterial)` members
  (`DecoratedShape.inner_shape`, `CompoundShape.SubShape.shape`, `ConvexShape.material`).
  They are released in `destruct`.
- Arrays owned by a shape use `self.base...base.allocator`.
- Every field has a default that a half-constructed shape can be destroyed with, because
  `initFromSettings` may stop at an error.

### D3 ShapeSettings, ShapeResult and shape construction

**ShapeSettings** is a pattern A RefTarget root with `cached_result: ShapeResult`. Its only
virtual function is
`createShape(self: *ShapeSettings, allocator) Allocator.Error!ShapeResult`. The receiver is
mutable because it writes the cache (Rule M). The returned copy belongs to the caller.

**ShapeResult** is `Result(Ref(Shape))` from `Zolt/Core/Result.zig`, a value type:
- `set`, `setError`, `setErrorFmt`, `assign`, `clone`, `deinit`, `isValid`, `hasError`,
  `getError`, `getPtr`.
- The error text lives inline (127 bytes; Jolt's longest collision message is 89), so caching
  and copying never allocate.
- `PhysicsMaterialResult`, `GroupFilterResult` and the other `Result` types follow the same
  pattern.

**Constructors.** A shape's C++ constructors become:
- `initDefault(allocator)`: the default constructor, used by restore and `createCached`.
- `initFromSettings(self, settings, result, allocator) Allocator.Error!void`: the
  `(settings, outResult)` constructor. It runs in place on the heap object, base part first.

The standard `Create()` is one line:

```zig
// See: ShapeSettings
pub fn createShape(self: *SphereShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
    return ShapeSettings.createCached(SphereShape, self, allocator);
}
```

`createCached` runs `if (cached_result.isEmpty()) try constructShape(T, settings, allocator);`
and then `return cached_result.clone()`. `constructShape` is
`Ref<Shape> shape = new T(*this, mCachedResult)` with Jolt's reference counting. It is also used
directly by settings with custom Create logic (StaticCompoundShapeSettings):

```zig
pub fn constructShape(comptime T: type, settings: anytype, allocator: Allocator) Allocator.Error!void {
    const base: *ShapeSettings = virtual.upcast(ShapeSettings, settings);
    errdefer base.cached_result.clear(); // Out of memory is not cached, a later call can succeed

    const shape = try allocator.create(T);
    shape.* = .initDefault(allocator);
    var ref = Ref(Shape).init(virtual.upcast(Shape, shape));
    defer ref.deinit();
    try shape.initFromSettings(settings, &base.cached_result, allocator);
}
```

The body of a constructor follows the C++ line by line. It writes Jolt's error text verbatim and
ends with `result.set(.init(self.asShapeMut()))`. A child error is forwarded with
`result.assign(&child_result)`.

**Child settings.** Settings that hold child settings store `Ref(ShapeSettings)`, not Jolt's
`RefConst`, because creating the child writes the child's cache (Rule M). Usage:

```zig
var result = try settings.asShapeSettings().createShape(allocator);
defer result.deinit();
if (result.hasError()) ... result.getError() ...   // "Invalid radius"
const shape: *Shape = result.getPtr().?;
```

`Create(TempAllocator &)` overloads become `createShapeWithTempAllocator(allocator,
temp_allocator)`. In that case `createShape` uses a `TempAllocatorMalloc` over `allocator`, as
Jolt does.

### D4 CollisionDispatch and ShapeFunctions: a comptime registry

Jolt's mutable static tables become one immutable `Registry`
(`CollisionDispatch.zig`: `collide_shape`, `cast_shape`, `shape_functions`), built at compile
time by replaying Jolt's registration:
- `Registry.build(registrations)` runs `init()` (CollisionDispatch::sInit). That fills every
  entry with `collideUnsupported` / `castUnsupported`, which panic when asserts are enabled. It
  then calls each type's `register(comptime r: *Registry)`.
- Each `register` is a line-by-line port of that class's `sRegister`. Later registrations
  override earlier ones exactly as in Jolt.
- `RegisterTypes.zig` lists the classes in the order of `RegisterTypes.cpp` lines 102-130, Jolt
  registrations only. The foundation creates every shape file (a stub with an empty `register`
  when not ported yet), so porters never edit this list.

```zig
pub const registry: Registry = .build(registration_order ++ user_types.registrations);

pub fn register(comptime r: *Registry) void { // TriangleShape::sRegister
    const f = r.shapeFunctions(.triangle);
    f.construct = ShapeFunctions.constructor(TriangleShape);
    f.color = Color.green;

    for (ShapeFile.convex_sub_shape_types) |s| {
        r.registerCollideShape(s, .triangle, collideConvexVsTriangle);
        r.registerCastShape(s, .triangle, castConvexVsTriangle);

        // Avoid registering triangle vs triangle as a reversed test to prevent infinite recursion
        if (s != .triangle) {
            r.registerCollideShape(.triangle, s, CollisionDispatch.reversedCollideShape);
            r.registerCastShape(.triangle, s, CollisionDispatch.reversedCastShape);
        }
    }
    ...
}
```

**Dispatch functions.** `collideShapeVsShape`, `castShapeVsShapeLocalSpace` / `WorldSpace`,
`reversedCollideShape` and `reversedCastShape` keep Jolt's signatures (`void`, no registry
parameter) and read `RegisterTypes.registry`.

**ShapeFunctions.** `ShapeFunctions.get(sub_type)` is read-only, with
`construct: ?*const fn (Allocator) Allocator.Error!*Shape` and `color`.

**User shapes** (User1..8, UserConvex1..8) and user materials come from the module
`zolt_user_types`:
- `build.zig` wires an empty default module. An application calls
  `zolt.addImport("zolt_user_types", mine)`; a cyclic module import works in Zig 0.16.
- The module may declare `registrations` (types with `register`, run after Jolt's, so they
  override), `default_material` and `material_types`.
- UserConvex shapes already get every convex function from `ConvexShape.register`. The prototype
  test builds a user registry and checks that it overrides.

### D5 Collectors

**`CollisionCollector(ResultType, Traits)`** is a pattern A root with the data in the base:
`vtable`, `early_out_fraction` and `context: ?*const TransformedShape`. The hot accessors are
therefore field reads, and `addHit` is one indirect call.
- VTable: `reset`, `onBody(*const Body)`, `onBodyEnd`, `setUserData`, and
  `addHit(*const ResultType) void`.
- Queries take `*CastRayCollector` (etc.), and callers pass `&collector.base`.

**Implementations.** Each `CollisionCollectorImpl` template is a comptime function whose result
embeds `base: CollectorType`:
- `init(...)` constructs the collector itself.
- `initDerived(T, ...)` constructs it as the base of a user class T, so T's vtable is used.
- Results that hold references (`TransformedShape`: `clone` / `deinit`) are cloned when stored
  and released when overwritten, on `reset` and on `deinit`.

A derived collector (Jolt's CastShapeTests does this):

```zig
const MyCollector = struct {
    pub const overrides = .{.addHit};

    base: ClosestHitPerBodyCollisionCollector(CastRayCollector),
    num_add_hit: u32 = 0,

    fn init(a: Allocator) @This() {
        return .{ .base = .initDerived(@This(), a) };
    }

    pub fn addHit(self: *@This(), result: *const RayCastResult) void {
        self.num_add_hit += 1;
        self.base.addHit(result); // C++ ClosestHitPerBodyCollisionCollector::AddHit(inResult)
    }
};
```

**Wrapping collectors** (`InternalEdgeRemovingCollector`, the reversed collectors, the
compound visitors' collectors) are local structs with `overrides`. They build their base with
`initFrom`, which copies the early out fraction and the context:
`var reversed_collector: ReversedCollector = .{ .base = .initFrom(ReversedCollector, collector), .collector = collector };`.

### D6 Out of memory inside queries

`addHit` stays `void`, and no query signature changes. A collector that allocates:
1. records the first error in an `AllocationErrorLatch`, which is never overwritten until
   `reset`;
2. calls `forceEarlyOut()` so the query stops as soon as possible;
3. reports it through `checkError()`, which the owner calls after the query.

In safe builds, `reset()` and `deinit()` assert that a recorded error was observed, so a
forgotten check fails in tests.

```zig
pub fn addHit(self: *Self, result: *const ResultType) void {
    var hit = copyResult(ResultType, result);
    self.hits.append(self.allocator, hit) catch |err| {
        releaseResult(ResultType, &hit);
        self.alloc_error.set(err);
        self.base.forceEarlyOut();
    };
}
```

For `ClosestHitPerBody`, `had_hit` stays false when storing the first hit of a body fails.
`onBodyEnd` only restores the early out fraction after a hit (Jolt line 147), so the forced early
out survives and the query stops.

Callers pass `&collector.base` to the query, then call `try collector.checkError()`.

### D7 Filters and listeners

**Filters** (`ShapeFilter`, `SimShapeFilter`, `BodyFilter`, `ObjectLayerFilter`,
`BroadPhaseLayerFilter`, `ObjectLayerPairFilter`, `ObjectVsBroadPhaseLayerFilter`,
`BroadPhaseLayerInterface`) use pattern A. Their virtual functions are const, users derive from
them, and Jolt passes a default-constructed base.
- `vtable` defaults to the base class's accept-everything table, so `ShapeFilter{}` (or `&.{}`)
  is Jolt's `{ }` default argument. Derived filters use `base: ShapeFilter = .init(@This())`.
- Shape queries and CollisionDispatch only read the filter (`*const ShapeFilter`).
- Entry points that write `body_id2` take a mutable pointer and default to a local filter, which
  satisfies Rule M:

```zig
pub fn castRayCollector(self: *const TransformedShape, ray: RRayCast, ray_cast_settings: *const RayCastSettings, collector: *CastRayCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
    if (self.shape.get()) |shape| {
        var default_filter: ShapeFilter = .{};
        const shape_filter = opts.shape_filter orelse &default_filter;

        // Set the context on the collector and filter
        collector.setContext(self);
        shape_filter.body_id2 = self.body_id;
```

These entry points are TransformedShape, NarrowPhaseQuery and the CharacterVirtual queries.
`ReversedShapeFilter` is a local `var` that copies `body_id2`.

**GroupFilter** is a pattern A RefTarget root like PhysicsMaterial.

**Listeners** (`ContactListener`, `BodyActivationListener`, `CharacterContactListener`, step
listeners) use pattern B from the guide: `ptr: *anyopaque` (their callbacks are non-const in
Jolt) and nullable entries for optional callbacks.

### D8 PhysicsMaterial and the default material

`PhysicsMaterial` is a pattern A RefTarget root. The base class constructor is
`init(T, allocator)`. `PhysicsMaterialSimple` owns a copy of its name, as Jolt's `String` does.

`PhysicsMaterial::sDefault` becomes a compile-time constant in read-only memory:

```zig
/// Default material that is used when a shape has no materials defined (PhysicsMaterial::sDefault)
pub const default: *const PhysicsMaterial = RegisterTypes.default_material;

pub fn addRef(self: *const PhysicsMaterial) void {
    if (!self.is_static) self.ref_count.addRef();
}
```

`RegisterTypes.default_material` is either `user_types.default_material` or
`&PhysicsMaterialSimple.default_material.base`, a
`PhysicsMaterialSimple.initStatic("Default", Color.grey)`. Static materials are never
reference counted, so `RefConst(PhysicsMaterial).init(PhysicsMaterial.default)` is legal and
nothing is mutable.

**Binary state.** It writes Jolt's RTTI hash. `rtti_name` is a vtable data entry, hashed with
`HashCombine.hashString` folded to 32 bits. `restoreFromBinaryState` finds the class in the
comptime `RegisterTypes.material_types` list until the Phase 8 Factory replaces it.

### D9 Support functions

**`ConvexShape.Support`** is a small pattern A root with entries `getSupport(direction)` and
`getConvexRadius()`. `Support.init(T)` refuses types with `deinit` or `destruct`: supports are
never destroyed, so they cannot own anything.

**`SupportBuffer`** is `PlacementBuffer(4160, 16, .{})`. `emplace(T)` checks size and alignment
at compile time and returns uninitialized storage, which the shape fills in place. This is C++
placement new: no 4 KB stack copy for ConvexHullShape, and self-referencing objects work.

```zig
switch (mode) {
    .include_convex_radius => {
        const support = buffer.emplace(SphereWithConvex);
        support.* = .init(scaled_radius);
        return &support.base;
    },
    ...
```

**GJK/EPA.** The already ported GJK and EPA take `*const Support` as their `anytype` convex
object, wrapped with `TransformedConvexObject` / `AddConvexRadius` as in Jolt. That is one
virtual call per support point, in the same operation order.

### D10 GetTrianglesContext and other caller buffers

**`Shape.GetTrianglesContext`** is `PlacementBuffer(4288, 16, .{ .type_check = true })`.
`getTrianglesStart` does `context.emplace(Ctx)` and initializes it in place;
`getTrianglesNext` does `context.get(Ctx)`, which asserts the type in safe builds.
- A context that points into itself, such as ConvexShape's context (a SupportBuffer plus a
  `*const Support` into it), uses an in-place `init(self: *Ctx, ...)`:
  `context.emplace(CSGetTrianglesContext).init(self, position_com, rotation, scale);`.
- Other contexts may assign by value:
  `context.emplace(GetTrianglesContextVertexList).* = .init(...)`.
- `getTrianglesNext` takes slices instead of counts: `out_triangle_vertices: []Float3` with
  `out_materials: ?[]*const PhysicsMaterial`. The dispatcher asserts their lengths.
- Static vertex tables that Jolt builds in static initializers, such as
  `ConvexShape.unit_sphere_triangles` and BoxShape's triangles, are comptime constants with the
  same bits; a test compares them with a runtime build.

Every other placement new into a caller-owned buffer uses `PlacementBuffer`.

### D11 Binary state

**Signatures.**
- `saveBinaryState(self: *const, stream: StreamOut) void`.
- `restoreBinaryState(self: *, stream: StreamIn) Allocator.Error!void`. The receiver is mutable,
  and the error union exists because restores read arrays with `StreamIn.readArray`.
- The material and sub shape state functions use the same `*const` / `*` split, and the save
  side of each takes an allocator.

**Write order.** Save and restore write the base class first. The C++ explicit base call
`ConvexShape::SaveBinaryState(inStream)` becomes `ConvexShape.impl.saveBinaryState(&self.base,
stream)`, in exactly Jolt's field order.

**`Shape.restoreFromBinaryState(allocator, stream) Allocator.Error!ShapeResult`.**
- It reads the sub type, rejects an invalid enum value or a null `ShapeFunctions.construct`
  with "Failed to read type id", constructs the shape, and restores it.
- On EOF or failure it returns "Failed to restore shape".

**Children.** `saveWithChildren` and `restoreWithChildren` use `ShapeToIDMap`, `MaterialToIDMap`,
`IDToShapeMap` and `IDToMaterialMap` (`Zolt/Core/ObjectToIDMap.zig`) with StreamUtils semantics.

### D12 Value types: TransformedShape, SubShapeID, ScaleHelpers

**`TransformedShape`** is a value type that owns `shape: RefConst(Shape)`, with fields in Jolt's
order (size asserted: 64 bytes, or 96 with double precision).
- `init` adds a reference, `clone()` is the copy constructor, `deinit()` the destructor, and `=`
  moves. Collectors clone the TransformedShapes they store.
- `getBodyID(?*const TransformedShape)` is Jolt's `sGetBodyID`.

**`SubShapeID`** and **`SubShapeIDCreator`** are `extern struct` values. `popID` returns
`PopResult{ id, remainder }`.

**`ScaleHelpers`** is a namespace of free functions.

### D13 Allocation during queries

- Queries take no allocator. Results go to the collector, which owns its allocator (D6).
- `JPH_STACK_ALLOC` and fixed local arrays become fixed arrays or `StaticArray` sized by Jolt's
  constants. Unbounded sizes use the caller's `TempAllocator`, which shape building already
  receives.
- Allocating operations take an `allocator` and return `Allocator.Error!...`: `createShape`,
  `restoreFromBinaryState`, `scaleShape`, `saveWithChildren` and `saveMaterialState`.
- Settings that own arrays (`CompoundShapeSettings.addShape`) use the allocator stored in the
  settings and also return `Allocator.Error`.

### D14 Layout and naming

- One Zig file per Jolt header, with the same path under `Zolt/`, e.g.
  `Zolt/Physics/Collision/Shape/SphereShape.zig`. Settings and shape live in the same file.
- Cyclic imports are fine: `Shape.zig` imports `ScaledShape.zig` and `StaticCompoundShape.zig`
  for `scaleShape`.
- Zolt-only helpers are `Zolt/Core/Virtual.zig` and `Zolt/Core/PlacementBuffer.zig`, with the
  header `//! Zolt addition, no Jolt file: ...`. `Zolt/Core/Result.zig` ports
  `Jolt/Core/Result.h`.
- Collector aliases (`CastRayCollector`, ...) and the map and list aliases are declared in
  `Shape.zig`.
- Names follow the guide. Jolt's non-virtual `ConvexShape::GetMaterial()` is renamed
  `getConvexMaterial` because the virtual function `getMaterial(sub_shape_id)` has the plain
  name. Static helpers registered in the dispatch table keep their name without `s`:
  `collideConvexVsConvex`, `castConvexVsConvex`.

## 2. Porter template: a concrete shape file

Copy `Zolt/Physics/Collision/Shape/SphereShape.zig` or `BoxShape.zig` (complete ports that follow this template).
Skeleton:

```zig
//! Port of: Jolt/Physics/Collision/Shape/XShape.h, Jolt/Physics/Collision/Shape/XShape.cpp
//! Status: complete | partial
//! Missing: <C++ functions not ported yet, e.g. JPH_DEBUG_RENDERER (Draw)>

pub const XShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,            // or DecoratedShapeSettings / CompoundShapeSettings / ShapeSettings
    field: f32 = <Jolt default>,

    pub fn initDefault(allocator: Allocator) XShapeSettings { ... }                 // default constructor
    pub fn init(allocator: Allocator, <C++ args>, opts: struct { <defaulted C++ args> }) XShapeSettings { ... }
    pub fn create(allocator: Allocator, ...) Allocator.Error!*XShapeSettings { ... } // new XShapeSettings(...)
    pub fn asShapeSettings(self: *XShapeSettings) *ShapeSettings { return &self.base.base; }
    pub fn deinit(self: *XShapeSettings) void { self.asShapeSettings().deinit(); }

    pub fn createShape(self: *XShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(XShape, self, allocator);
    }
};

pub const XShape = struct {
    pub const shape_sub_type: ShapeSubType = .x;
    pub const overrides = .{ <every C++ `override` of XShape, in header order> };

    base: ConvexShape,
    field: f32 = 0.0,                       // valid default: half constructed shapes are destroyed normally

    pub fn initDefault(allocator: Allocator) XShape {
        return .{ .base = .init(XShape, allocator, shape_sub_type, null) };
    }

    pub fn initFromSettings(self: *XShape, settings: *const XShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base, result);   // base constructor first
        ...                                                    // C++ body, Jolt's error texts
        result.set(.init(self.asShapeMut()));
    }

    pub fn init(allocator: Allocator, ...) XShape { ... }                     // other C++ constructors
    pub fn create(allocator: Allocator, ...) Allocator.Error!*XShape { ... }  // new XShape(...)
    pub fn destruct(self: *XShape) void { ... }                               // only if it owns refs / arrays
    pub fn asShape(self: *const XShape) *const Shape { return &self.base.base; }
    pub fn asShapeMut(self: *XShape) *Shape { return &self.base.base; }

    // Non virtual functions, then the overrides (`// See Shape::Foo` comments as in Jolt)
    pub fn getLocalBounds(self: *const XShape) AABox { ... }
    ...

    pub fn register(comptime r: *Registry) void {      // XShape::sRegister, line by line
        const f = r.shapeFunctions(.x);
        f.construct = ShapeFunctions.constructor(XShape);
        f.color = Color.<Jolt color>;
    }

    const XNoConvex = struct {                          // support classes (D9)
        pub const overrides = .{ .getSupport, .getConvexRadius };
        base: ConvexShape.Support,
        ...
    };
};
```

Variants:
- **Decorated shapes** use `base: DecoratedShape` and `.init(T, allocator, shape_sub_type,
  inner)`. They start with `try self.base.initFromSettings(&settings.base, result, allocator);`
  followed by `if (result.hasError()) return;`.
- **Compound shapes** use `base: CompoundShape` and override `getIntersectingSubShapes*`.
- **Shapes with arrays** (Mesh, HeightField, ConvexHull, compounds) allocate with
  `self.base.base.allocator` and free in `destruct`.
- **Shapes that build with a temp allocator** expose `createShapeWithTempAllocator` (D3).

## 3. Checklist for porting a shape

1. **Header.** `Port of`, `Status`, `Missing`. Keep Jolt's doc and implementation comments.
2. **Settings.** Field defaults are Jolt's; provide `initDefault` / `init` / `create`;
   `createShape` is `createCached`, or custom code ending in `constructShape`.
3. **Overrides.** `overrides` lists exactly the C++ `override`s that are ported, in header
   order. A function not ported yet falls back to the base class, so list it under `Missing`.
4. **Signatures.** Const C++ methods take `*const Self`; mutable ones (`restore*`, setters) take
   `*Self`. Out parameters become `*T` or a returned struct, and overloads get the suffixes from
   D1.
5. **Calls.** Unqualified virtual calls use the dispatcher unless the class is `final`; `Base::`
   calls use `Base.impl.foo` (D1). Never `@constCast` a write (Rule M).
6. **Construction.** `initFromSettings` writes Jolt's error strings verbatim, uses `try` for
   allocations, and calls `result.set` last.
7. **Lifetime.** `RefConst` members and owned arrays are released in `destruct`; heap objects
   come from `create`.
8. **Placement.** Support and GetTriangles contexts use `emplace(T)` with in-place init. Static
   tables are comptime constants.
9. **Binary state.** Base first, Jolt's field order, `try` on reads that allocate, using the
   shape's allocator.
10. **Registration.** Port `sRegister` into the existing stub `register`, then set `Status`.
    Export public types in `Zolt/zolt.zig` (the file itself was registered by the foundation).
11. **Unit tests.** Port the matching Jolt tests (same names and values) into
    `ZoltTests/Physics/...`, using `std.testing.allocator`. Add a `FailingAllocator` loop over
    `createShape` for shapes that allocate (see the prototype's "out of memory" test).
12. **Parity.** Add `ZoltParity/` tests for everything that computes numbers: bounds, mass
    properties, support points, castRay / collide / cast results, getTriangles, inner radius,
    volume, surface normals and supporting faces. Compare bit for bit in both precisions.
13. **Finish.** Run `zig build test`, `-Ddouble_precision=true`, `-Doptimize=ReleaseFast`,
    `zig build parity`, `zig fmt --check ...` and `python3 tools/port_status.py --write`.

## 4. Things that are easy to get wrong

- **ConvexShape fallbacks.** ConvexShape's `castRay`, `castRayCollector` and `collidePoint` are
  GJK fallbacks. A shape whose C++ overrides them must list them, or it silently loses bit
  exactness. The `overrides` checks catch typos but not omissions, so compare against the
  header.
- **ShapeFilter with compounds.** The ID of the last sub shape of a compound can equal the empty
  ID, e.g. index 1 of 2 sub shapes, which is all ones. This is Jolt behavior; do not "fix" it.
- **Collector context.** A collector's `context` is only valid during `addHit` (Jolt's note).
  Never store it; store a `clone()` of the TransformedShape instead.
- **Settings ownership.** `Ref(ShapeSettings)` children of heap settings are released with the
  parent. Stack settings referenced by heap settings must be `setEmbedded()`.

## 5. Phase 4 port order

**F1 Foundation, part 1** (one worker; nothing else starts before it is merged):
- `Core/Virtual.zig`, `Core/PlacementBuffer.zig` and `Core/Result.zig`, from the prototype.
- `Physics/Body/BodyID.zig`, `Physics/Body/MassProperties.zig` (complete), a minimal
  `Physics/Body/Body.zig` stub, and `Physics/PhysicsSettings.zig` constants.
- `ObjectLayer`, `BroadPhaseLayer`, `BackFaceMode`, `ActiveEdgeMode`, `CollectFacesMode`,
  `SubShapeID`, `SubShapeIDPair`, `RayCast`, `AABoxCast`, `CastResult`,
  `CollidePointResult`, `CollideShape`, `ShapeCast`, `SortReverseAndStore` (and remove the
  private copies in the AABB tree tests).
- `CollisionCollector`, `CollisionCollectorImpl`, `ShapeFilter`, `SimShapeFilter`,
  `PhysicsMaterial` and `PhysicsMaterialSimple`.

**F2 Foundation, part 2** (one worker):
- `Shape.zig` with the complete VTable, including every virtual function the prototype omits
  (GetSubmergedVolume, CollideSoftBodyVertices, GetSubShapeTransformedShape,
  GetWorldSpaceBounds(DMat44), Draw* behind the debug renderer option).
- The abstract bases `ConvexShape`, `DecoratedShape`, `CompoundShape` and
  `CompoundShapeVisitors`, plus `ScaleHelpers`, `GetTrianglesContext`, `CollisionDispatch`,
  `TransformedShape` and `RegisterTypes.zig`.
- A stub file with an empty `register` and `Status: stub` for every shape in the registration
  order, including `SoftBodyShape`, whose file belongs to Phase 9.
- The `zolt_user_types` module in `build.zig` and the guide updates (section 7).
- `tools/port_status.py`: remove `Jolt/Core/Result` from `NOT_APPLICABLE`.
  (Done: the foundation is merged and the prototype removed.)

**Wave A** (parallel, one file per worker):
- Convex shapes, each including its support classes: `SphereShape`, `BoxShape`, `CapsuleShape`,
  `TaperedCapsuleShape`, `CylinderShape`, `TaperedCylinderShape`, `ConvexHullShape` (+
  `PolyhedronSubmergedVolumeCalculator`), `PlaneShape`, `EmptyShape`.
- Decorated shapes: `ScaledShape`, `RotatedTranslatedShape`, `OffsetCenterOfMassShape`.
- The triangle algorithms: `ActiveEdges`, `ManifoldBetweenTwoFaces`,
  `CollideConvexVsTriangles`, `CollideSphereVsTriangles`, `CastConvexVsTriangles`,
  `CastSphereVsTriangles`, `CollideShapeVsShapePerLeaf`, `InternalEdgeRemovingCollector`.
  `CollideSoftBodyVertexIterator` / `CollideSoftBodyVerticesVsTriangles` may stay stubs until
  Phase 9 (SoftBody).
- Filters and groups: `GroupFilter`, `GroupFilterTable`, `CollisionGroup`,
  `ObjectLayerPairFilterMask` / `Table`, `BroadPhaseLayerInterfaceMask` / `Table`,
  `ObjectVsBroadPhaseLayerFilterMask` / `Table`, and the `ContactListener` types.

**Wave B** (each needs Wave A pieces):
- `TriangleShape`, which needs the triangle algorithms.
- `StaticCompoundShape` and `MutableCompoundShape`, which need RotatedTranslatedShape for the
  single-sub-shape case.
- `MeshShape` and `HeightFieldShape`, which need the triangle algorithms and the AABB tree.

**Phase 5, not Phase 4:** `BroadPhase`, `BroadPhaseQuery`, `QuadTree`, `BroadPhaseQuadTree`,
`BroadPhaseBruteForce`, `NarrowPhaseQuery`, `EstimateCollisionResponse` and `SimShapeFilterWrapper`
depend on `Body` / `BodyManager` and are ported with them instead of against stubs.
`ManifoldBetweenTwoFaces.cpp` includes `ContactConstraintManager.h` only for its debug draw flags
(`sDrawContactPoint...`, behind `JPH_DEBUG_RENDERER`), so it belongs in Wave A. `ContactListener`
needs only the `Body` pointer type from the F1 stub.

## 6. Judges' critical flaws and how this design resolves them

| Flaw | Resolution |
|---|---|
| `mutable` members written through `@constCast` of `*const` (cache, `body_id2`): UB, lost in ReleaseFast | Rule M. `createShape` takes `*ShapeSettings`; entry points take `?*ShapeFilter`; regression test "Rule M" calls through a `noinline` vtable call and runs in ReleaseFast |
| `@hasDecl` misses private or misspelled overrides, so the base version is used silently | mandatory `overrides` list plus the `impl` namespace; every case is a compile error (D1 table, all verified) |
| builder ignores overrides of a concrete intermediate class | `make` searches every level; `initDerived`; test "a collector derived from ClosestHitPerBody inherits its overrides" |
| `restoreBinaryState` returned `void` | `Allocator.Error!void`; test restores a compound with an allocation failure |
| `emplace(T, value)` copies 4 KB objects; self-referencing contexts unsafe | `emplace(T) *T` with in-place init (D9, D10) |
| `ShapeResult` replaced by error unions, `QueryError` everywhere | Jolt's `Result` value API; `addHit` stays `void` with the latch (D3, D6) |
| mutable global default material | read-only constant with `is_static` (D8) |
| atomic refcount through `*const` | the single documented Rule M exception; ReleaseFast tests |
| borrowed material name | `PhysicsMaterialSimple` owns a copy; test frees the caller's string |
| user registration and default material through `@import("root")` (unusable in tests) | `zolt_user_types` module (D4) |
| non-Jolt specialization in the registry | `registration_order` contains only Jolt classes; a user registry is built only in a test |
| listeners declared with `ptr: *const anyopaque` | pattern B with `*anyopaque` (D7) |
| `ClosestHit.reset` leaked a stored reference | `reset` releases the hit |
| ClosestHitPerBody undid the forced early out after OOM | `had_hit` stays false, so `onBodyEnd` keeps the forced early out; the latch keeps the error and the check is asserted (test "out of memory") |

## 7. Porting guide updates (done in F2)

- **Section 6 (pattern A).** Replace the hand-written vtable example with D1:
  `virtual.make`, `overrides` / `impl`, prefix vtables, dispatchers, the call rules, the
  `destruct` chain and data entries.
- **Section 6 (pattern B).** Mention `*anyopaque` for non-const callbacks.
- **Section 5, Allocation.** Add the exception to "Result<T> becomes an error union": Jolt's
  cached and copied results (`ShapeResult` and its relatives) stay `Result(T)` values; only
  allocation failure is a Zig error.
- **New rule M** (section 5 or 9). Never write through memory reached from a `*const`
  parameter, with the two exceptions from section 0. List `@constCast` writes in review.
- **Section 5, Reference counting.** Cover static (`is_static`) and embedded RefTargets,
  `create` + `Ref.init`, and the allocator stored in the root.
- **Section 5.** Document `PlacementBuffer` for placement new, and comptime tables for static
  initializers.
- **Section 10.** Shape tests run with `std.testing.allocator`, a `FailingAllocator` loop on
  creation, and in ReleaseFast.

## 8. Open questions

- Phase 5 must use the Rule M rules for body and constraint settings that hold `mutable`
  caches (same pattern as D3).
- `Body::sFixedToWorld` probably becomes a static constant with an `is_static` RefTarget flag,
  like the default material.
- `GetStats` sizes differ from C++ because every root stores an `Allocator` (16 bytes). No Jolt
  test compares them.
- `setErrorFmt` uses Zig formatting. Messages with `%g` (ConvexHullShape) need a helper that
  formats like C's `%g` to stay text-identical.
- `RefConst` to `Ref` conversion: `StaticCompoundShapeSettings` stores
  `@constCast(shape_ptr)` in its result as Jolt does (`const_cast`). Only the atomic count is
  written through it.
- A lint (grep in CI) for `@constCast` writes and for non-pub functions with virtual names
  would complement the compile-time checks.

## 9. Settled during the foundation port

- **Foundation steps actually used:** F1 (infrastructure and basic types), shape core (Shape,
  collectors, filters, dispatch, TransformedShape, RegisterTypes with a stub per shape), filters, and
  the abstract bases in their own steps (ConvexShape with SphereShape and BoxShape; DecoratedShape,
  CompoundShape and CompoundShapeVisitors). Wave A and B start from the merged foundation.
- **Holding a concrete shape.** Concrete shapes have no `addRef`/`release` of their own: Jolt's
  `RefConst<SphereShape>` is `RefConst(Shape)` plus `shape.cast(SphereShape)`.
- **GetTrianglesNext** counts (`max_triangles_requested`, the returned count) are `u32` (Jolt `int`).
- **Allocating `save*` functions** (`saveMaterialState`, `saveSubShapeState`, `getStatsRecursive`)
  take an allocator and return `Allocator.Error`; reserve capacity before cloning a reference into a
  list (`ensureUnusedCapacity` + `appendAssumeCapacity`) so out of memory never leaks a reference.
- **Compound visitors.** Walkers take `visitor: anytype` and call `shouldAbort`, `testBounds` (Vec4
  distances for the ray and cast visitors, UVec4 masks for the others) and `visitShape`. Jolt's
  derived visitors (`struct Visitor : public CastRayVisitor`) embed the base visitor as `base` and
  forward the protocol (`CompoundShapeVisitors.zig` header).
- **Registered types from the user module** (`zolt_user_types`): `registrations`, `default_material`,
  `material_types` and `group_filter_types` (`RegisterTypes.zig`).
- **Test-only shapes.** `Zolt/Physics/Collision/Shape/TestShapes.zig` (only compiled in tests) registers
  User1..3 test shapes and a test material for the library's inline tests; the parity build has its
  own user types module (`ZoltParity/Physics/ShapeCoreUserTypes.zig`). Wave A/B parity tests use the
  real shapes instead of adding user shapes.
- **Parity C++ wrappers** that need shapes create the Factory and call `RegisterTypes()` once per
  binary through the guard `if (Factory::sInstance == nullptr)` (see `BasicsReference.cpp`).
- **Shared test helpers:** `ZoltTests/Layers.zig` (port of `UnitTests/Layers.h`).
- **`CollideSoftBodyVertexIterator`** has the operations shapes need for `CollideSoftBodyVertices`
  (Status: partial until SoftBody, Phase 9).
- **NarrowPhaseStats** follow Jolt's default (`JPH_TRACK_NARROWPHASE_STATS` off): a comptime switch.

**Settled during Waves A and B:**
- `restoreMaterialState` returns `Allocator.Error!void` (D11): MeshShape and HeightFieldShape assign
  their material lists.
- Query-like free functions whose Jolt implementation allocates through an `STLLocalAllocator` heap
  fallback take an `allocator` and return `Allocator.Error` (exception to D13):
  `InternalEdgeRemovingCollector.collideShapeVsShape` and `collideShapeVsShapePerLeaf`.
  `InternalEdgeRemovingCollector` itself is built in place (`c.init(chained, tolerance_sq,
  allocator)`, `deinit`, `checkError`) because it contains its local buffers.
- A settings `createShape` that builds a different shape type (TaperedCylinderShapeSettings with equal
  radii builds a CylinderShape, TaperedCapsuleShapeSettings) builds it from temporary settings and
  assigns the result into its own cache, like Jolt; it drops what Jolt drops (density, user data).
- Jolt's lambda registrations (EmptyShape's collide/cast functions) become named private functions so
  the registry can store and compare them.
- A virtual whose C++ body is only `JPH_ASSERT(false)` and whose out parameters became a returned
  struct panics when asserts are on and returns zeros otherwise.
- Jolt's own debug asserts that degenerate input reaches (CollideConvexVsTriangles on degenerate
  triangles, AnyHit collectors behind InternalEdgeRemovingCollector, WalkSubShapes' early abort) fire in
  Zolt's safe builds exactly as in an assert-enabled Jolt build; parity generators avoid those inputs,
  and ReleaseFast parity compares Jolt's release behavior.
- Jolt bugs reproduced on purpose (parity confirms the same bits; report upstream rather than fix):
  HeightFieldShape::GetTrianglesNext never finishes when one leaf block has more triangles than
  requested; HeightFieldShape skips empty range blocks only through inside-out Y bounds, which collapse
  for a flat height field far from the origin; ConvexHullShape's CastRayHelper uses
  `mPoints[*end_vtx]` (probably meant `*(end_vtx - 1)`), so hulls of exactly 3 points miss an edge;
  TaperedCylinderShapeSettings drops density and user data in its equal-radii shortcut; ScaledShape's
  MakeScaleValid can return a scale below `ScaleHelpers::cMinScale`.
