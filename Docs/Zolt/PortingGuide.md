# Zolt Porting Guide (C++ Jolt → Zig 0.16)

This is the rulebook for porting Jolt Physics to Zig. Follow it for every file so that the port
stays consistent, reviewable and **bit exact** with the C++ library. When a situation is not
covered here, pick the option closest to the C++ code, then add the rule to this file.

Contents:
1. [Goals](#1-goals)
2. [Workflow for porting a file](#2-workflow-for-porting-a-file)
3. [Layout and naming](#3-layout-and-naming)
4. [Functions, operators and overloads](#4-functions-operators-and-overloads)
5. [Types, memory and ownership](#5-types-memory-and-ownership)
6. [Polymorphism (virtual functions)](#6-polymorphism-virtual-functions)
7. [Templates and macros](#7-templates-and-macros)
8. [Floating point determinism](#8-floating-point-determinism)
9. [Threading](#9-threading)
10. [Tests](#10-tests)
11. [Zig 0.16 notes and pitfalls](#11-zig-016-notes-and-pitfalls)
12. [Rename table](#12-rename-table)

---

## 1. Goals

1. **Faithful port.** Same algorithms, same data structures, same order of floating point
   operations as Jolt. A Zolt simulation must produce the *same bits* as Jolt built with
   `CROSS_PLATFORM_DETERMINISTIC=ON`. The end-to-end acceptance test is reproducing the hashes
   in `.github/workflows/determinism_check.yml` (see [Roadmap](Roadmap.md)).
2. **Idiomatic where it is free.** Zig naming, explicit allocators, error unions instead of
   `Result<T>`, comptime instead of templates/macros. Never change behavior for idiom's sake.
3. **Traceable.** Every Zig file says which C++ files it ports. Doc comments are kept, so the
   Jolt documentation stays valid for Zolt.

## 2. Workflow for porting a file

1. Pick the next file from [Progress.md](Progress.md) / [Roadmap.md](Roadmap.md); its
   dependencies should already be ported (stub what is missing, see "Status" below).
2. Read the C++ with ISA branches stripped:
   ```sh
   python3 tools/strip_isa.py -D JPH_CROSS_PLATFORM_DETERMINISTIC Jolt/Math/Vec3.inl
   ```
   The portable fallback (`#else` of `JPH_USE_SSE` / `JPH_USE_NEON` / ...) defines the semantics
   (except in the rare places where it differs from the SSE path, see section 8 rule 11).
   Express it with `@Vector` operations rather than lane-by-lane loops.
3. Merge `X.h` + `X.inl` + `X.cpp` into `Zolt/<same dir>/X.zig`, starting with the header:
   ```zig
   //! Port of: Jolt/Math/Vec3.h, Jolt/Math/Vec3.inl, Jolt/Math/Vec3.cpp
   //! Status: complete
   ```
   `Status` is `complete` (everything ported), `partial` (some declarations missing; list them
   in a `//! Missing:` line) or `stub` (declarations only, so dependents can compile).
   `tools/port_status.py` reads these headers to generate `Progress.md`.
4. Register the file in `source_files` in `Zolt/zolt.zig` and re-export its public types there.
5. Port the matching tests from `UnitTests/` into `ZoltTests/` (see [Tests](#10-tests)).
6. Add parity tests (see [Tests](#10-tests)): every ported function that computes numbers gets
   compared bit for bit with the C++ library on many inputs.
7. Run `zig build test`, `zig build test -Ddouble_precision=true`, `zig build parity` (also with
   `-Ddouble_precision=true` when the file touches `Real` / `RVec3`), `zig fmt Zolt ZoltTests ZoltParity build.zig`,
   then `python3 tools/port_status.py --write`.

## 3. Layout and naming

### Files

| C++                                    | Zig                                                |
|----------------------------------------|----------------------------------------------------|
| `Jolt/<Dir>/<Name>.h/.inl/.cpp`        | `Zolt/<Dir>/<Name>.zig` (one file)                 |
| `UnitTests/<Dir>/<Name>Test(s).cpp`    | `ZoltTests/<Dir>/<Name>Test(s).zig`                |
| `#include <Jolt/Math/Vec3.h>`          | `const Vec3 = @import("../Math/Vec3.zig").Vec3;`   |
| `namespace JPH`                        | the `zolt` module; public API re-exported flat in `Zolt/zolt.zig` |

Each Zig file is a *namespace* that holds the same declarations as the C++ header (a header
often has several classes), e.g. `Vec3.zig` declares `pub const Vec3 = extern struct {...}`.
Inside the library always import by relative path, never through `zolt.zig`.

### Identifiers

| C++                                   | Zig                                   | Example                                       |
|---------------------------------------|---------------------------------------|-----------------------------------------------|
| class / struct                        | same name                             | `BodyInterface` → `BodyInterface`             |
| `enum class EFoo { Bar }`             | `enum { bar }`, drop the `E`          | `EMotionType::Dynamic` → `MotionType.dynamic` |
| member function `GetFoo()`            | camelCase                             | `GetLinearVelocity` → `getLinearVelocity`     |
| static function `sFoo()`              | drop `s`, camelCase                   | `Vec3::sZero()` → `Vec3.zero()`               |
| member variable `mFoo`                | snake_case, drop `m`                  | `mLinearVelocity` → `linear_velocity`         |
| static member variable `sFoo`         | snake_case `pub var`                  | `sDrawConstraints` → `draw_constraints`       |
| constant `cFoo` / `constexpr`         | snake_case `const`                    | `cLargeFloat` → `large_float`                 |
| parameter `inFoo`, `outFoo`, `ioFoo`  | snake_case, drop prefix               | `inVelocity` → `velocity`                     |
| macro constant `JPH_FOO`              | snake_case `const`                    | `JPH_PI` → `math.pi`                          |
| type alias `using Foo = Bar`          | `pub const Foo = Bar;`                |                                               |

- A parameter may not shadow a declaration in Zig (e.g. a `set` parameter in a struct that has a
  `set()` method). Append `_value`: `sSelect(inNotSet, inSet, ...)` → `select(not_set_value, set_value, ...)`.
- Identifiers that are Zig keywords get a descriptive name, see the [rename table](#12-rename-table).
  Use `@"name"` only as a last resort.

## 4. Functions, operators and overloads

- **Operators** become methods:

  | C++              | Zig                         |
  |------------------|-----------------------------|
  | `a + b`          | `a.add(b)`                  |
  | `a - b`          | `a.sub(b)`                  |
  | `a * b`          | `a.mul(b)` (component wise) |
  | `a * s`, `s * a` | `a.mulScalar(s)`            |
  | `a / b`          | `a.div(b)`                  |
  | `a / s`          | `a.divScalar(s)`            |
  | `-a`             | `a.negate()`                |
  | `a += b`         | `a = a.add(b)`              |
  | `a == b`         | `a.eql(b)`, `!=` is `!a.eql(b)` |
  | `a[i]` (read)    | `a.getComponent(i)`         |
  | `a[i] = x`       | `a.setComponent(i, x)`      |
  | `M * v`          | `m.mulVec3(v)` / `m.mulVec4(v)`, `m.mul(m2)` for matrices |
  | `ostream <<`     | `pub fn format(self, writer: *std.Io.Writer) std.Io.Writer.Error!void` (print with `{f}`) |

  Keep the operand order of the C++ expression; it matters for floating point (see section 8).
- **Overloads** get distinct names with a suffix describing what differs: `mulScalar`,
  `fromVec3`, `fromVec3W`. Constructors: the primary one is `init`, others are `fromX`.
- **Default arguments**: trailing parameters with defaults become a final anonymous options struct
  with the same defaults. Callers pass `.{}` or override by name:
  ```zig
  pub fn isClose(self: Vec3, other: Vec3, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool
  // a.isClose(b, .{})   a.isClose(b, .{ .max_dist_sq = 1.0e-6 })
  ```
- **Out parameters**: one out parameter → return value. Several → return a named struct
  (`Vec4.sinCos() SinCos{ .sin, .cos }`). In/out (`io`) parameters and large outputs that are
  filled incrementally → pointer parameter.
- **const methods** take `self: T` for small value types (vectors, IDs) and `self: *const T`
  for everything else. Mutating methods take `self: *T`.
- **`inline`**: do not mark functions `inline` by default, even when Jolt uses `JPH_INLINE`.
  LLVM inlines small functions in release builds anyway, and Zig only type-checks an inline
  function where it is called, which hides compile errors in code without tests. Use `inline`
  only when needed for comptime (e.g. `inline for` over comptime data) or when a measurement
  shows it helps.
- `[[nodiscard]]`: nothing to do, Zig already refuses to ignore return values.

## 5. Types, memory and ownership

### Primitive types

| C++                                  | Zig                                         |
|--------------------------------------|---------------------------------------------|
| `uint8/16/32/64`, `int8/16/32/64`    | `u8/u16/u32/u64`, `i8/i16/i32/i64`          |
| `uint` / `int`                       | `u32` / `i32` (use `u32`/`usize` for values that are only ever non-negative indices) |
| `size_t`                             | `usize`                                     |
| `float` / `double`                   | `f32` / `f64`                               |
| `Real`, `RVec3`, `RMat44`            | `Real`, `RVec3`, `RMat44` from `Math/Real.zig` (switch on `-Ddouble_precision`) |

Code that uses `RVec3` / `RMat44` must compile in both precisions, but the mixed-type overloads
have different names (`DMat44 * DVec3` is `mulDVec3`, `Mat44 * Vec3` is `mulVec3`). Use the
precision independent spellings, which exist on both types: `RMat44.mulRVec3`,
`multiply3x3RVec3`, `preTranslatedRVec3`, `postTranslatedRVec3`, and `RVec3.addVec3` /
`subVec3` for `RVec3 +/- Vec3`. Add new ones the same way (a `pub const xRVec3 = ...;` alias on
both types) when porting code needs them.
| `nullptr`                            | `null` (with `?*T`)                         |

### Value types and layout

- Plain data that Jolt copies by value (Vec3, AABox, BodyID, ...) is a `struct`. Use
  `extern struct` when layout matters: SIMD types, data that is hashed/streamed as bytes,
  data shared with GPU code. Uninitialized C++ values (`Vec3 v;`) become `var v: Vec3 = undefined;`.
- `static_assert(sizeof(X) == N)` → `comptime { std.debug.assert(@sizeOf(X) == N); }`.

### Allocation

- There is no global allocator. Anything that allocates takes a `std.mem.Allocator`
  (named `allocator`), either per call or once in `init` (stored in the struct when `deinit`
  needs it). Jolt's `Allocate` / `Free` / `AlignedAllocate` hooks and `JPH_OVERRIDE_NEW_DELETE`
  disappear.
- Construction/destruction: `T.init(...) T` / `T.init(allocator, ...) !T` with `deinit(self: *T)`
  for values; `T.create(allocator, ...) !*T` with `destroy()` for heap objects.
- Types that hand out pointers to themselves during construction (worker threads, jobs that store
  the job system) cannot be returned by value. They have a default constructed `pub const empty`
  and initialize in place: `var pool: JobSystemThreadPool = .empty; try pool.init(allocator, io, ...)`
  (Jolt's default constructor + `Init`). Document that they must not be moved after `init`.
- `TempAllocator` is ported as its own type (it is a stack allocator with LIFO semantics), and
  additionally exposes a `std.mem.Allocator` interface for use with std containers.
- `JPH_STACK_ALLOC(n)` (alloca): use a fixed size array when `n` is comptime known, otherwise a
  fixed stack buffer with an asserted upper bound (see `Math/GaussianElimination.zig`), or the
  `TempAllocator` when the size is unbounded. Zig has no alloca.
- Allocation failure is an error (`error.OutOfMemory`), never ignored. Where Jolt returns a
  `Result<T>` or an error string, return `!T` with a specific error set; keep Jolt's message text
  in a doc comment or log it with `std.log`.

### Containers

| C++ (Jolt)                       | Zig                                                                   |
|----------------------------------|-----------------------------------------------------------------------|
| `Array<T>`                       | `std.ArrayList(T)` (unmanaged in 0.16: pass the allocator to every mutating call) |
| `StaticArray<T, N>`              | port of `Core/StaticArray.h` (std has no BoundedArray anymore)        |
| `UnorderedMap`, `UnorderedSet`, `HashTable` | port of Jolt's `Core/HashTable.h` when iteration order can affect results, otherwise `std.HashMapUnmanaged`: `UnorderedMap(Key, Value, .{})`, `UnorderedSet(Key, .{})`, the `Hash` / `KeyEqual` template arguments become `HashTableOptions(Key){ .hash, .key_equal }` |
| `QuickSort`, `InsertionSort`     | ports of `Core/QuickSort.h` / `Core/InsertionSort.h` (std::sort / std.sort orders differ, which breaks determinism for equal keys) |
| `String`, `string_view`          | `[]const u8` (owned strings: `[]u8` + allocator)                      |
| `std::pair<A, B>`                | named struct                                                          |
| `std::function`                  | function pointer + `*anyopaque` context, or a comptime `anytype` callback when the call site is static. When the function is stored (a job's function, a thread init/exit callback): an inline closure like `JobSystem.JobFunction` / `JobSystemThreadPool.InitExitFunction`. `JobFunction.init(comptime function, args_tuple)` stores a copy of the arguments (at most 4 words and at most `usize` aligned, checked at compile time; capture larger or more aligned data by pointer) and never allocates. A lambda capture becomes the tuple: by reference → a pointer, by value → a copy (`[&values, i] { ... }` → `.init(f, .{ &values, i })`) |
| iterator pairs `(inBegin, inEnd)` | a slice; comparators follow std.sort: `context` + `lessThan(context, a, b)` (see `Core/QuickSort.zig`) |

Container methods use the std.ArrayList names, for `std.ArrayList` and Zolt's own array-like
containers alike (the hash containers are below): `push_back` → `append`, `pop_back` → `pop`,
`size()` → `.len` / `.items.len`, `empty()` → `len == 0` / `isEmpty()`, `erase(it)` →
`orderedRemove(i)`, `reserve` → `ensureTotalCapacity`, `clear` → `clearRetainingCapacity`
(ArrayList) / `clear` (StaticArray), `back()` → `getLast()` (ArrayList) / `back()` (StaticArray), `begin()`/`end()`/`data()` → `.items` / `slice()`.

The hash containers (`HashTable`, `UnorderedMap`, `UnorderedSet`, see `Core/HashTable.zig`) follow the std hash
map names instead: `size()` → `count()`, `empty()` → `isEmpty()`, `reserve` → `ensureTotalCapacity`,
**`clear()` → `clearAndFree(allocator)`** (Jolt's `clear` frees the buckets; do not use `clearRetainingCapacity`
for it: the bucket count decides where elements land and therefore the iteration order),
`ClearAndKeepMemory()` → `clearRetainingCapacity()`, `operator[]` → `getOrPutValue(allocator, key, default)`,
`try_emplace` → `tryEmplace`, const `find` → `find` (`?*const KeyValue`, null is `end()`), non-const `find` →
`findPtr`, `erase(it)` → `eraseByPtr(ptr)`, `it.mIndex` → `indexOf(ptr)`, `begin()`/`end()` loops →
`iterator()` / `constIterator()` with `next()`, `std::pair` `first` / `second` → `KeyValue{ .key, .value }`.
Copies (`clone` / `assign`) are bitwise: values that own memory must be duplicated by the caller.

### Reference counting

`RefTarget<T>` / `Ref<T>` / `RefConst<T>` keep Jolt's intrusive reference count, but without RAII
(implemented in `Core/Reference.zig`):
- the target embeds `ref_count: RefCount` and declares `addRef()` / `release()`; `release()`
  destroys the object when `self.ref_count.release()` returns true (see the example at the top of
  `Core/Reference.zig`). Memory orders are Jolt's (`.monotonic` add, `.acq_rel` release);
- a `Ref<T>` / `RefConst<T>` member becomes `Ref(T)` / `RefConst(T)`: `init(ptr)` / `set(ptr)` add a
  reference like the C++ constructor / assignment, and the owner calls `deinit()` where the C++
  destructor would run. Raw pointers stay raw (`*T`) where Jolt uses raw pointers;
- `new Foo(...)` assigned to a `Ref` becomes `Foo.create(allocator, ...)` (refcount 0) followed by
  `Ref(Foo).init(ptr)`; the object keeps its allocator to destroy itself.

### Strings, I/O and logging

- `Trace(fmt, ...)` → `std.log.scoped(.zolt).info/warn(...)`; applications control output via
  `std_options.logFn`.
- `StreamIn` / `StreamOut` are interfaces (pattern B, `Core/StreamIn.zig` / `Core/StreamOut.zig`);
  `StreamInWrapper` / `StreamOutWrapper` adapt a `*std.Io.Reader` / `*std.Io.Writer` (flush the
  writer yourself). The `Read`/`Write` overloads become one `read(&value)` / `write(value)` that
  dispatches at comptime (Vec3 = 12 bytes, DVec3 = 24, DMat44 = 72, everything else raw bytes,
  which must be trivially copyable: extern/packed structs, no pointers, checked at compile time).
  Reads keep the in/out parameter, because a validating `StateRecorder` compares with the current
  value. Arrays and strings: `readArray` / `readString` / `readArrayWith` and `writeArray` /
  `writeString` / `writeArrayWith`.
- Jolt's `String` results (`ConvertToString`, `StringFormat`, ...) become owned `[]u8` returned
  from a function that takes the allocator. `StringFormat` takes a Zig format string, not printf.

## 6. Polymorphism (virtual functions)

Use one of two patterns. Both keep Jolt's ability to add user-defined subclasses.

**A. Class hierarchies with data in the base class** (`Shape`, `ConvexShape`, `Constraint`,
`ShapeSettings`, `BroadPhase`, ...): the base struct holds a `vtable: *const VTable` plus its
data; each derived struct embeds its parent as the first field named `base`; downcasting uses
`@fieldParentPtr` (one step per level of the hierarchy).
```zig
pub const Shape = struct {
    pub const VTable = struct {
        getLocalBounds: *const fn (self: *const Shape) AABox,
        // ... one entry per C++ virtual function, in declaration order
    };
    vtable: *const VTable,
    ref_count: std.atomic.Value(u32) = .init(0),
    user_data: u64 = 0,

    pub fn getLocalBounds(self: *const Shape) AABox {
        return self.vtable.getLocalBounds(self);
    }
};

pub const ConvexShape = struct {
    base: Shape,
    density: f32 = 1000,
};

pub const BoxShape = struct {
    base: ConvexShape,
    half_extent: Vec3,

    const vtable: Shape.VTable = .{ .getLocalBounds = getLocalBoundsImpl };

    pub fn init(half_extent: Vec3) BoxShape {
        return .{ .base = .{ .base = .{ .vtable = &vtable } }, .half_extent = half_extent };
    }

    /// Downcast (static_cast<const BoxShape *>(shape) in C++)
    pub fn fromShape(shape: *const Shape) *const BoxShape {
        const convex: *const ConvexShape = @alignCast(@fieldParentPtr("base", shape));
        return @alignCast(@fieldParentPtr("base", convex));
    }

    fn getLocalBoundsImpl(shape: *const Shape) AABox {
        const self = fromShape(shape);
        return .{ .min = self.half_extent.negate(), .max = self.half_extent };
    }
};
```
Virtuals with a default implementation in the base: the base exposes the default as a `pub fn`
that derived vtables can reference. Jolt's switch-on-subtype dispatch tables (e.g.
`CollisionDispatch`) are ported as tables, exactly like the C++.

**B. Pure interfaces / listeners** (`ContactListener`, `BodyActivationListener`,
`BroadPhaseLayerInterface`, `ObjectLayerPairFilter`, `JobSystem`, ...): a type-erased fat pointer
like `std.mem.Allocator`, with a vtable generated at comptime from any implementation type.
Optional virtuals (with an empty/default C++ body) are nullable entries, so implementations only
declare the callbacks they need:
```zig
pub const ContactListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        onContactAdded: ?*const fn (ptr: *anyopaque, body1: *const Body, body2: *const Body) void = null, // null = Jolt's default (empty) implementation
    };

    /// Wrap any `*T` that declares some of the callbacks
    pub fn init(impl: anytype) ContactListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            // Thunks need their own names: Zig forbids shadowing `onContactAdded` of the outer struct
            fn onContactAddedThunk(ptr: *anyopaque, body1: *const Body, body2: *const Body) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onContactAdded(body1, body2);
            }
            const vtable: VTable = .{
                .onContactAdded = if (@hasDecl(T, "onContactAdded")) onContactAddedThunk else null,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    pub fn onContactAdded(self: ContactListener, body1: *const Body, body2: *const Body) void {
        if (self.vtable.onContactAdded) |f| f(self.ptr, body1, body2);
    }
};
```

Exception: a pure interface whose identity is its address uses pattern A (the implementation embeds
`base: Iface`, which holds the vtable, and hands out `*Iface`), because a fat pointer does not fit in
the atomic integer Jolt stores it in. Example: `JobSystem.Barrier` (a `Job` keeps its barrier in a
`std.atomic.Value(usize)`), implemented by `JobSystemWithBarrier.BarrierImpl`.

**Visitors / templated callbacks** (`template <class Visitor> void Walk(Visitor &)`) become
`anytype` parameters, which keeps static dispatch like the C++.

## 7. Templates and macros

| C++                                         | Zig                                                         |
|---------------------------------------------|-------------------------------------------------------------|
| `template <class T> class Foo`              | `pub fn Foo(comptime T: type) type { return struct {...}; }` |
| `template <int N> void f()`                 | `fn f(comptime n: i32) void` (or `comptime_int`)            |
| `Swizzle<SWIZZLE_Y, SWIZZLE_X, ...>()`      | `.swizzle(.y, .x, ...)` (comptime enum parameters)          |
| `JPH_ASSERT(x)` / `JPH_ASSERT(x, "msg")`    | `std.debug.assert(x)` (keep the message as a comment)       |
| `JPH_ASSERT(false)` on a path that release builds can reach and handle (e.g. "too many iterations", then `return false`) | `if (Core.enable_asserts) @panic("msg");` followed by the release behavior: `assert(false)` is undefined behavior in ReleaseFast |
| `JPH_IF_ENABLE_ASSERTS(x)`                  | `if (Core.enable_asserts) { x }`                            |
| `JPH_IF_DEBUG(x)` / `#ifdef JPH_DEBUG`      | `if (builtin.mode == .Debug)`                               |
| `#ifdef JPH_DOUBLE_PRECISION`               | `if (Core.double_precision)` (comptime known)               |
| `#ifdef JPH_DEBUG_RENDERER`                 | not ported yet: leave `// TODO(debug_renderer): ...` where code is skipped |
| developer debug switches that are commented out in Jolt (`JPH_GJK_DEBUG`, `JPH_EPA_PENETRATION_DEPTH_DEBUG`, `JPH_EPA_CONVEX_BUILDER_DRAW`, `JPH_EPA_CONVEX_BUILDER_VALIDATE`, `JPH_CONVEX_BUILDER_DEBUG`, `JPH_CONVEX_BUILDER_DUMP_SHAPE`, `JPH_CONVEX_BUILDER_2D_DEBUG`, ...) | not ported: list them in a `//! Not ported: ...` header line. They do not affect the Status (a file without them is `complete`) |
| `#ifdef JPH_CPU_BIG_ENDIAN`                 | `if (builtin.cpu.arch.endian() == .big)` (comptime); port both branches |
| `JPH_PROFILE(...)`, `JPH_PROFILE_FUNCTION()`| dropped for now (a profiler may come later)                 |
| `JPH_DET_LOG(...)`                          | port when the determinism log is needed for debugging       |
| `JPH_NAMESPACE_BEGIN/END`, `JPH_EXPORT`, `JPH_SUPPRESS_WARNINGS*` | dropped                               |
| `JPH_DECLARE_SERIALIZABLE_*`, `JPH_RTTI`    | deferred to the ObjectStream phase; leave a `// TODO(serialization)` |
| `JPH_CACHE_LINE_SIZE` alignment             | `align(Core.cache_line_size)`                               |
| `static_assert`                             | `comptime { std.debug.assert(...); }` or `@compileError`    |
| `JPH_MAKE_HASHABLE(T, a, b)`                | `pub fn getHash(self: T) u64 { return HashCombine.hashCombineArgs(.{ a, b }); }` |

## 8. Floating point determinism

Zolt follows Jolt built with `JPH_CROSS_PLATFORM_DETERMINISTIC` and **without**
`JPH_FLOATING_POINT_EXCEPTIONS_ENABLED`, i.e. the configuration that produces the reference
hashes in `determinism_check.yml`. Zig does no fast-math and no FP contraction by default, so
results are reproducible as long as the code follows these rules:

1. **Keep the operation order.** Never reassociate, factor or "simplify" floating point
   expressions. Translate the C++ expression tree literally: `(a * b + c) * d` must stay
   `a.mul(b).add(c).mul(d)`. Watch C++ precedence and left-to-right evaluation in long chains.
2. **No FMA.** Never use `@mulAdd`; `sFusedMultiplyAdd` / `DifferenceOfProducts` are plain
   `a * b + c` / `a * b - c * d` in deterministic mode.
3. **Vector negation is `0 - x`**, which maps -0 to +0 (`Vec3`/`Vec4`/`DVec3` `negate()`, Jolt's
   `operator-()` in deterministic mode). Scalar negation in the C++ (`-a * b`, `-s`) is a real sign
   flip: keep it as `-x` in Zig.
4. **Horizontal sums** use the deterministic order: Vec4 `(x + y) + (z + w)`, Vec3 `(x + y) + (z + 0)`.
5. **Trigonometry**: always `Math/Trigonometry.zig` / `Vec4.sinCos` etc., never `std.math.sin`.
   `@sqrt` is fine (IEEE correctly rounded everywhere).
6. **Scalar min/max/clamp**: C++ `min(a, b)` / `max(a, b)` on floats → `math.min` / `math.max` /
   `math.clamp` from `Math/Math.zig` (they reproduce `std::min/max`, unlike `@min/@max`, which
   differ for NaN and -0). Integer `@min` / `@max` are fine. Vector `min`/`max` follow `_mm_min_ps`.
7. **Approximations are exact**: `MulRSqrtApproximate` is `a / sqrt(b)` in deterministic mode.
8. **Constants**: copy float literals verbatim from the C++ source.
9. Code under `JPH_FLOATING_POINT_EXCEPTIONS_ENABLED` (e.g. adding `FLT_MIN` to avoid a division
   by zero) is **not** ported. Vec3 keeps W == Z regardless, so its padding lane is always defined.
10. Wherever Jolt uses `std::mt19937`, use `Core/Mt19937.zig` (bit identical sequence). Never use
    `std.Random` for anything that affects simulation results.
11. **SIMD path vs scalar fallback.** In a few places Jolt's SSE path does not produce the same bits as
    its scalar fallback. There Zolt follows the SSE path, which is what `zig build parity` compares
    against (and what Jolt computes on x86, usually also on ARM). Known cases: `Mat44::Inversed` (SSE,
    NEON and RVV share an algorithm that differs from the fallback), `Mat44::sCrossProduct` (SSE4.1
    negates with `0 - v`, the fallback with `-x`), `Vec3/Vec4/DVec3::Abs` (SSE/AVX compute
    `max(0 - v, v)`, which keeps -0, while the fallback, NEON and AVX512 return +0). Jolt's ISA paths
    sometimes disagree with each other on -0 / NaN only; the determinism hashes are identical across
    platforms, so these cases don't affect simulation results. The parity reference is pinned to
    x86-64-v3 (SSE4.2/AVX2, Jolt's default CMake ISA set, no AVX512) so that it is the same on every
    machine. Differences that only show with AVX512 on NaN input (`Vec3/DVec3::GetSign` return NaN)
    are not followed; the parity tests skip NaN there.

## 9. Threading

Zig 0.16 moved blocking synchronization into the `std.Io` interface (`std.Thread.Mutex`,
`Condition`, `Semaphore`, `ResetEvent`, `Pool` and `WaitGroup` no longer exist):
- **Where `io` comes from.** A type that blocks (locks, waits) gets an `io: std.Io` in `init`, next to
  the allocator, and stores it, e.g. `pool.init(allocator, io, ...)` for `JobSystemThreadPool`. Applications get
  it from `std.process.Init` (`init.io`) or `std.Io.Threaded`; tests use `std.testing.io`.
- **Small sync primitives take `io` per call** instead of storing it, because they are embedded in many
  other structs (Jolt's `Mutex` is a member of BodyManager, MutexArray, ...): `mutex.lock(io)`,
  `mutex.unlock(io)`, `mutex.tryLock()`.
- **Physics code is not cancelable.** Use the `*Uncancelable` variants of `std.Io` primitives
  (`lockUncancelable`, `waitUncancelable`) so that lock/wait functions keep Jolt's signatures
  (no error union). Expose `Cancelable` errors only where Jolt itself has a failure path.
- Mapping: `std::mutex` / Jolt `Mutex` → Zolt `Core/Mutex.zig` `Mutex` over `std.Io.Mutex`;
  `std::shared_mutex` / Jolt `SharedMutex` → `Core/Mutex.zig` `SharedMutex` over `SharedMutexBase`
  (a copy of `std.Io.RwLock` with a fixed `tryLock`). Do not use `std.Io.RwLock` directly: its
  `tryLock` in Zig 0.16 can succeed while a reader holds the lock; `std::condition_variable` →
  `std.Io.Condition`; Jolt `Semaphore` (counting, `Acquire(n)` / `Release(n)`) → port of
  `Core/Semaphore.zig` on top of `std.Io.Mutex` + `std.Io.Condition` or atomics + `std.Io` futex
  (`io.futexWait` / `io.futexWake`), keeping Jolt's fast path (atomic counter, only block when needed).
- `std::atomic<T>` → `std.atomic.Value(T)`; memory orders: `relaxed` → `.monotonic`,
  `acquire`/`release`/`acq_rel`/`seq_cst` map 1:1. Zig has no standalone fence (`@fence` is gone):
  where Jolt uses `atomic_thread_fence`, strengthen the adjacent atomic operation instead and comment why.
- `std::thread` → `std.Thread.spawn(.{}, func, .{args})` + `join()`; `std::this_thread::yield()` →
  `std.Thread.yield()`; cache line padding → `align(std.atomic.cache_line)` / `Core.cache_line_size`.
  Zig reorders the fields of a (non-extern) struct, so `alignas(JPH_CACHE_LINE_SIZE)` on one member
  does not keep the members declared before it off its cache line. When Jolt uses it to separate
  groups of members (false sharing), put the group in a nested struct whose first field is
  `align(Core.cache_line_size)` (it then occupies whole cache lines) and check the offsets at comptime,
  see `FixedSizeFreeList.free_list`.
- `JobSystem` is interface pattern B; `JobSystemThreadPool` is built on `std.Thread` + `std.Io`
  primitives. Multithreaded results must be identical to single threaded ones (Jolt guarantees
  this, so the port must keep the same barriers and sorting of results).
- Debug-only lock checking in Jolt (`JPH_ENABLE_ASSERTS` lock tracking) maps to `Core.enable_asserts`.

## 10. Tests

- Ported Jolt tests live in `ZoltTests/` mirroring `UnitTests/`, one file per C++ file, and are
  registered in `ZoltTests/unit_tests.zig`. They use only the public API (`@import("zolt")`).
- Test names are the C++ `TEST_CASE` names: `TEST_CASE("TestVec3Cross")` → `test "TestVec3Cross"`.
  Run one with `zig build test -Dtest-filter=TestVec3Cross`.
- `CHECK(x)` → `try expect(x)`; `CHECK(a == b)` on numbers → `try expectEqual(b, a)` (Zig puts the
  expected value first); `CHECK_APPROX_EQUAL(a, b, tol)` → `try checkApproxEqual(a, b, .{ .tolerance = tol })`
  from `ZoltTests/UnitTestFramework.zig`.
- Keep the exact input values and expected values of the C++ test.
- `ExpectAssert` (tests that trigger asserts) cannot be ported: Zig asserts panic. Skip those
  checks with a comment `// Not ported: relies on ExpectAssert`.
- Small tests of Zolt-specific code (helpers that have no Jolt counterpart) go inline in the
  source file as `test` blocks.
- **Parity tests** (`ZoltParity/`, run with `zig build parity`) are the proof of exactness.
  `build.zig` compiles the C++ library from `Jolt/` with Zig's C++ compiler in the configuration
  Zolt follows (`JPH_CROSS_PLATFORM_DETERMINISTIC`, `-ffp-contract=off`, same precision and layer
  bits, CPU pinned to x86-64-v3) and links it into the parity test binary. Layout mirrors `Zolt/`:
  `ZoltParity/<Dir>/<Dir>Parity.zig` holds the tests and `ZoltParity/<Dir>/<Dir>Reference.cpp` the
  C ABI wrappers around Jolt; shared helpers (input generator, bit comparison, `Checker`) are in
  `ZoltParity/ParityFramework.zig`. Register test files in `ZoltParity/parity.zig` and .cpp files in
  `ZoltParity/reference_sources.zig`. For each ported function, call both implementations on ~100k
  generated inputs (random values mixed with special values) and require identical bits (NaN
  payloads excepted). For containers and algorithms with an observable order (hash table iteration,
  sorting with equal keys, heaps), compare the order. A parity mismatch is always a porting bug
  unless Jolt's own ISA paths disagree (see section 8, rule 11). Higher level code (collision
  queries, simulation steps) is compared the same way, e.g. by hashing body state after N steps.
- Do not pass `bool` parameters across the C ABI in parity wrappers, use `int`: declare the
  parameter `int inFoo` in the .cpp (use it as `inFoo != 0`) and `foo: c_int` in the `extern fn`
  (pass `@intFromBool(foo)`). Zig 0.16 (LLVM backend) does not always zero-extend runtime `bool`
  arguments, so the clang-compiled callee can receive 0xFE for `false` and read it as `true`.
  `bool` return values and `bool` fields or out parameters behind a pointer are fine.

- `Zolt/zolt.zig` references every public declaration of every registered file, so all
  non-generic functions are type checked even without a test. Generic functions need a test.

## 11. Zig 0.16 notes and pitfalls

- Vectors can only be indexed with comptime-known indices. For runtime indices copy to an array:
  `const a: [4]f32 = v.value; return a[i];`.
- `@Type` is gone; use `@Int(.unsigned, bits)`, `@Struct(...)`, etc.
- `std.ArrayList(T)` is unmanaged (`.empty`, `append(allocator, x)`, `deinit(allocator)`);
  `std.array_list.Managed` exists but don't use it.
- `std.Thread.Mutex` / `Condition` / `Semaphore` no longer exist: see [Threading](#9-threading).
- `std.io` is now `std.Io` (`std.Io.Writer`, `std.Io.Reader`); format methods are printed with `{f}`.
- `main` receives `std.process.Init` (`init.gpa`, `init.io`, `init.arena`).
- `usingnamespace` and `async` are gone. `std.BoundedArray` is gone.
- Zig 0.16's self-hosted x86_64 backend (Zig's default for Debug builds) miscompiles some `@Vector`
  code, e.g. `@bitCast(@Vector(4, bool))` to `u4` returns garbage. `build.zig` therefore defaults to
  LLVM (`-Duse_llvm=false` opts out for faster Debug compiles). Avoid bitcasting bool vectors to
  integers; build masks with shifts instead (see `UVec4.getTrues`).
- Integer arithmetic that wraps on purpose (hashes, counters) must use `+%`, `-%`, `*%`;
  plain operators are overflow-checked in Debug/ReleaseSafe.
- `std.math.nan(f32)` is a quiet NaN like `numeric_limits<float>::quiet_NaN()`.
- Shifts need a right operand of the exact log2 type: `x << @intCast(n)` with `n` a `u5` for `u32`.

## 12. Rename table

Names that cannot be ported mechanically. Add to this table whenever you pick a new name.

| Jolt                               | Zolt                                   | Reason                                  |
|------------------------------------|----------------------------------------|-----------------------------------------|
| `sAnd`, `sOr`, `sXor`, `sNot`      | `bitAnd`, `bitOr`, `bitXor`, `bitNot`  | `and`/`or` are keywords                 |
| `sSelect(inNotSet, inSet, ctrl)`   | `select(not_set_value, set_value, ctrl)` | `set` parameter shadows `set()`       |
| `Vec4::SinCos(outSin, outCos)`     | `sinCos() SinCos{ .sin, .cos }`        | out parameters                          |
| `Vec3(Vec4Arg)`                    | `Vec3.fromVec4`                        | overload                                |
| `Vec3(const Float3 &)`             | `Vec3.fromFloat3`                      | overload                                |
| `Vec4(Vec3Arg)` / `Vec4(Vec3Arg, w)` | `Vec4.fromVec3` / `Vec4.fromVec3W`   | overload                                |
| `Hash<T>{}(v)`                     | `HashCombine.hash(v)`                  | functor template → comptime dispatch     |
| `HashBytes(data, size, seed)`      | `hashBytes(data)` / `hashBytesSeeded(data, seed)` | default argument            |
| `std::mt19937`                     | `Mt19937`                              | std type, see Core/Mt19937.zig          |
| `JPH_PI`                           | `math.pi`                              |                                         |
| `Matrix<R, C>` / `Vector<R>` template params | `Matrix(r, c).row_count` / `.col_count`, `Vector(n).row_count` | comptime decls usable by generic code |
| `Matrix::operator()(row, col)` (also Mat44, DynMatrix) | `get(row, col)` / `set(row, col, v)` | operator overload            |
| `Matrix::GetColumn` (non-const reference) | `getColumnPtr(i)`                 | reference return                        |
| copy constructor of an allocating type (`DynMatrix(const DynMatrix &)`) | `clone() !T` | needs the allocator            |
| `FindRoot(a, b, c, outX1, outX2) -> int` | `findRoot(T, a, b, c) FindRootResult(T){ num_roots, x1, x2 }` | out parameters      |
| `JPH_EVS_ROTATE` (macro)           | private fn `evsRotate`                 | macro                                   |
| `DVec3::cTrue` / `cFalse`          | `DVec3.true_value` / `DVec3.false_value` | keywords                              |
| `DVec3(Vec3Arg)` / `DVec3(Vec4Arg)` / `DVec3(const Double3 &)` / `DVec3(TypeArg)` | `fromVec3` / `fromVec4` / `fromDouble3` / `fromType` (`fromType` sets W = Z) | overloads |
| `explicit operator Vec3()`         | `DVec3.toVec3()`                       | conversion operator                     |
| `DVec3 + Vec3`, `DVec3 - Vec3`     | `addVec3`, `subVec3`                   | overloads                               |
| `BVec16(uint64, uint64)`           | `BVec16.fromUint64(v0, v1)`            | overload                                |
| `Quat(const Float4 &)` / `Quat(Vec4Arg)` | `Quat.fromFloat4` / `Quat.fromVec4` | overloads                               |
| `Quat::GetAxisAngle(outAxis, outAngle)` | `getAxisAngle() AxisAngle{ .axis, .angle }` | out parameters                   |
| `Quat::GetSwingTwist(outSwing, outTwist)` | `getSwingTwist() SwingTwist{ .swing, .twist }` | out parameters             |
| `Quat::LERP` / `SLERP`             | `lerp` / `slerp`                       | naming                                  |
| `Quat * Vec3`                      | `mulVec3`                              | operator                                |
| `Mat44(Vec4, Vec4, Vec4, Vec3)` / `Mat44(Type x4)` | `Mat44.fromColumnsTranslation` / `Mat44.fromTypes` | overloads           |
| `Mat44::sRotation(axis, angle)` / `sRotation(QuatArg)` | `rotation` / `rotationQuat`  | overloads                               |
| `Mat44::sScale(float)` / `sScale(Vec3Arg)` | `scale` / `scaleVec3`          | overloads                               |
| `Mat44::Multiply3x3(Vec3Arg)` / `Multiply3x3(Mat44Arg)` | `multiply3x3` / `multiply3x3Mat44` | overloads               |
| `Mat44::Decompose(outScale)`       | `decompose() Decomposition{ .rotation_translation, .scale }` | out parameter     |
| `JPH_EL(r, c)` (macro)             | private `el(comptime r, comptime c)`   | macro                                   |
| duplicate `TEST_CASE` names in one file | second one gets a `2` suffix (`TestMat44Scale2`) | Zig rejects duplicate test names |
| `HalfFloatConversion::FromFloat<ROUND_TO_NEAREST>(v)` | `half_float.fromFloat(.round_to_nearest, v)` (file re-exported as `zolt.half_float`) | namespace + template |
| `HALF_FLT_MAX` etc.                | `half_float.half_flt_max` etc.         | constants                               |
| `FLT_MIN`, `FLT_MAX`, `FLT_EPSILON`| `math.flt_min`, `math.flt_max`, `math.flt_epsilon` |                             |
| `explicit DMat44(Mat44Arg)` / `DMat44(Mat44Arg inRot, DVec3Arg inT)` / `DMat44(Type x3, DTypeArg)` | `DMat44.fromMat44` / `fromMat44Translation` / `fromTypes` | overloads |
| `DMat44::sRotation(QuatArg)` / `sScale(Vec3Arg)` | `rotationQuat` / `scaleVec3` (same names as Mat44, so `RMat44` code works in both precisions) | overloads |
| `DMat44 * DMat44` / `DMat44 * Mat44` | `mul` / `mulMat44`                   | operator overloads                      |
| `DMat44 * Vec3` / `DMat44 * DVec3` | `mulVec3` / `mulDVec3` (both return DVec3) | operator overloads                 |
| `DMat44::Multiply3x3(DVec3Arg)`, `PreTranslated(DVec3Arg)`, `PostTranslated(DVec3Arg)` | `multiply3x3DVec3`, `preTranslatedDVec3`, `postTranslatedDVec3` (the `Vec3Arg` overloads keep Mat44's names) | overloads |
| `DMat44::Decompose(outScale)`      | `decompose() Decomposition{ .rotation_translation, .scale }` | out parameter     |
| `JPH_RVECTOR_ALIGNMENT`            | `rvector_alignment` (`Math/Real.zig`)  | macro constant                          |
| `operator ""_r` (`JPH::literals`)  | not ported: a float literal coerces to `Real` | Zig has no user-defined literals |
| `FixedSizeFreeList()` + `Init(inMaxObjects, inPageSize)` | `init(allocator, io, max_objects, page_size)` | constructor + Init; pages are allocated (and the page mutex locked) in `constructObject` |
| `FixedSizeFreeList::DestructObject(Object *)` | `destructObjectPtr(object)` (`destructObject(index)` keeps the name) | overload |
| `FixedSizeFreeList` `mPageMutex`, `mNumFreeObjects`, `mAllocationTag`, `mFirstFreeObjectAndTag`, `mFirstFreeObjectInNewPage` | `free_list.page_mutex`, ... | grouped in a cache line aligned struct (field reordering) |
| `LockFreeHashMap::KeyValue::GetValue() const` | `getValueConst()` (`getValue()` returns `*Value`) | const overload |
| `LockFreeHashMap::Iterator` `operator*` / `operator++` | `get()` / `advance()`, plus `next() ?*KeyValue` for `while (it.next()) \|kv\|` | operators |
| `LFHMAllocatorContext::Allocate(inSize, inAlignment, outWriteOffset) -> bool` | `allocate(size, alignment) ?u32` | out parameter |
| `mAllocator` (`LFHMAllocator &` in `LFHMAllocatorContext` / `LockFreeHashMap`) | `lfhm_allocator` | `allocator` is the `std.mem.Allocator` |
| `LockFreeHashMap(LFHMAllocator &)` + `Init(inMaxBuckets)` | `init(allocator, &lfhm_allocator, max_buckets)` | constructor + Init |
| `JobSystemThreadPool()` + `Init(inMaxJobs, inMaxBarriers, inNumThreads)` | `var pool: JobSystemThreadPool = .empty;` + `try pool.init(allocator, io, max_jobs, max_barriers, .{ .num_threads = n })` | constructor + Init, in place (the threads keep a pointer to the pool) |
| `JobSystemWithBarrier(inMaxBarriers)` / `JobSystemSingleThreaded(inMaxJobs)` (or constructor + `Init`) | `init(allocator, io, max_barriers)` / `init(allocator, io, max_jobs)` | constructor + Init |
| `JobHandle` copy constructor / copy assignment | `clone()` / `set(&other)` (a plain assignment moves, `deinit()` is the destructor) | no copy constructors |
| `JobHandle::sRemoveDependencies(const JobHandle *, uint)` / `sRemoveDependencies(StaticArray<JobHandle, N> &)` | `removeDependencies(slice, .{})` / `removeDependenciesStaticArray(&array, .{})` | overloads |
| `JobSystem::JobFunction` / `JobSystemThreadPool::InitExitFunction` (`std::function`) | `JobFunction.init(f, .{ args })` / `InitExitFunction.init(f, .{ args })` (calls `f(args..., thread_index)`), invoked with `call()` | closure (see the containers table) |
| `Job::mReferenceCount`             | `ref_count: RefCount`                  | same as RefTarget (reference counting)  |
| `JobSystemWithBarrier::BarrierImpl` `mJobReadIndex` / `mJobWriteIndex`, `mNumToAcquire` | `read_state.job_read_index` / `write_state.job_write_index`, `write_state.num_to_acquire` | grouped in cache line aligned structs (field reordering) |
| `JobSystemThreadPool::mTail`       | `tail_state.tail`                      | cache line aligned struct (field reordering) |
| `HashTable<..., Hash, KeyEqual>`, `UnorderedMap<Key, Value, Hash, KeyEqual>`, `UnorderedSet<Key, Hash, KeyEqual>` | `HashTable(Key, KeyValue, Detail, options)`, `UnorderedMap(Key, Value, options)`, `UnorderedSet(Key, options)` with `options: HashTableOptions(Key)` (`.{}` = `Hash<Key>` / `std::equal_to`) | functor template parameters |
| `HashTable::clear()` / `ClearAndKeepMemory()` | `clearAndFree(allocator)` / `clearRetainingCapacity()` | std names; `clear` frees the buckets (iteration order depends on the bucket count) |
| `HashTable::size()` / `empty()` / `reserve(n)` | `count()` / `isEmpty()` / `ensureTotalCapacity(allocator, n)` | std hash map names |
| `UnorderedMap::operator[](key)`    | `getOrPutValue(allocator, key, default_value)` | operator, `Value()` has no generic Zig equivalent |
| `UnorderedMap::try_emplace(key, args...)` | `tryEmplace(allocator, key, value)` | variadic constructor arguments |
| `find(key)` (non-const / const)    | `findPtr(key) ?*KeyValue` / `find(key) ?*const KeyValue` (null is `end()`) | overload on const |
| `HashTable::erase(const_iterator)` / `erase(key)` | `eraseByPtr(ptr)` / `erase(key)` | overload            |
| `HashTable` iterator `mIndex`      | `indexOf(ptr)`                         | iterators are element pointers          |
| `HashTable::begin()` / `end()`     | `iterator()` / `constIterator()` + `next()` (null at the end) | iterators              |
| copy / move constructor, `operator=` (copy / move) of `HashTable` / `UnorderedMap` | `clone(allocator)` / `move()`, `assign(allocator, &other)` / `assignMove(allocator, &other)` (copies are bitwise) | needs the allocator |
| `std::pair<const Key, Value>` (`first` / `second`) in `UnorderedMap` | `KeyValue{ .key, .value }` | named struct                     |
| `StreamUtils::ObjectToIDMap<T>` / `IDToObjectMap<T>` | `zolt.ObjectToIDMap(T)` / `zolt.IDToObjectMap(T)` | namespace flattened                |
| `TempAllocator::Allocate(inSize) -> void *` | `allocate(size) Error!?Block` (null for size 0, `error.OutOfMemory` where Jolt aborts) | error union instead of abort |
| `STLTempAllocator<T>`              | `STLTempAllocator` (untyped), containers use `.allocator()` (a `std.mem.Allocator`) | std.mem.Allocator counts bytes |
| `STLLocalAllocator(const STLLocalAllocator<T2, N> &)` | `STLLocalAllocator(T, N).fromOther(other)` | converting constructor  |
| `STLLocalAllocator::is_local`      | `isLocal`                              | STL style snake_case name               |
| `ByteBuffer::Align`                | `alignTo`                              | `align` is a keyword                    |
| `ByteBuffer::Allocate<T>(inSize = 1)` | `allocate(allocator, T, .{ .size = n }) ![]T` | default argument, returns a slice |
| `ByteBuffer::Get<T>` (non-const)   | `getMut(T, position)`                  | const overload                          |
| `StridedPtr<const T>`              | `StridedPtrConst(T)`                   | Zig types have no const qualifier       |
| `StridedPtr` `++p` / `--p` / `p++` / `p--` | `increment` / `decrement` / `postIncrement` / `postDecrement` | operators |
| `StridedPtr` `p - q` / `*p`, `p->` / `p[i]` | `distance` / `deref` / `at` (pointers) | operators                         |
| `StreamIn::Read(T &)` / `StreamOut::Write(const T &)` overloads | `read(&value)` / `write(value)` (comptime dispatch) | overloads                   |
| `StreamIn::Read(Array<T> &)` / `Read(String &)` / `Read(Array<T> &, F)` | `readArray(T, allocator, &list)` / `readString(allocator, &string)` / `readArrayWith(T, allocator, &list, context, readElement)` | overloads, need the allocator |
| `StreamOut::Write(Array<T>)` / `Write(String)` / `Write(Array<T>, F)` | `writeArray(T, items)` / `writeString(string)` / `writeArrayWith(T, items, context, writeElement)` | overloads |
| `ReadBytes(void *, size_t)` / `WriteBytes(const void *, size_t)` | `readBytes([]u8)` / `writeBytes([]const u8)` | pointer + size become a slice |
| `StreamInWrapper(istream &)` / `StreamOutWrapper(ostream &)` | `StreamInWrapper.init(*std.Io.Reader).streamIn()` / `StreamOutWrapper.init(*std.Io.Writer).streamOut()` | std streams become std.Io |
| `StringToVector(str, out, delim = ",", clear = true)` / `VectorToString(v, out, delim = ",")` | `stringToVector(allocator, str, &list, .{ .delimiter, .clear_vector })` / `vectorToString(allocator, v, .{ .delimiter }) ![]u8` | default arguments, out parameter |
| `FPControlWord<Value, Mask>` (RAII) | `const cw = FPControlWord(value, mask).init(); defer cw.deinit();` | RAII                                 |
| `LinearCurve` copy constructor     | `clone(allocator)`                     | allocating type                         |
