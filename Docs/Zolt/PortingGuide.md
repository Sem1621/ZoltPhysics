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
   The portable fallback (`#else` of `JPH_USE_SSE` / `JPH_USE_NEON` / ...) defines the semantics.
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
6. Run `zig build test` (and `zig build test -Ddouble_precision=true` if the file touches
   `Real` / `RVec3`), `zig fmt Zolt ZoltTests build.zig`, then `python3 tools/port_status.py --write`.

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
- `TempAllocator` is ported as its own type (it is a stack allocator with LIFO semantics), and
  additionally exposes a `std.mem.Allocator` interface for use with std containers.
- Allocation failure is an error (`error.OutOfMemory`), never ignored. Where Jolt returns a
  `Result<T>` or an error string, return `!T` with a specific error set; keep Jolt's message text
  in a doc comment or log it with `std.log`.

### Containers

| C++ (Jolt)                       | Zig                                                                   |
|----------------------------------|-----------------------------------------------------------------------|
| `Array<T>`                       | `std.ArrayList(T)` (unmanaged in 0.16: pass the allocator to every mutating call) |
| `StaticArray<T, N>`              | port of `Core/StaticArray.h` (std has no BoundedArray anymore)        |
| `UnorderedMap`, `UnorderedSet`, `HashTable` | port of Jolt's `Core/HashTable.h` when iteration order can affect results, otherwise `std.HashMapUnmanaged` |
| `QuickSort`, `InsertionSort`     | ports of `Core/QuickSort.h` / `Core/InsertionSort.h` (std::sort / std.sort orders differ, which breaks determinism for equal keys) |
| `String`, `string_view`          | `[]const u8` (owned strings: `[]u8` + allocator)                      |
| `std::pair<A, B>`                | named struct                                                          |
| `std::function`                  | function pointer + `*anyopaque` context, or a comptime `anytype` callback when the call site is static |

### Reference counting

`RefTarget<T>` / `Ref<T>` / `RefConst<T>` keep Jolt's intrusive reference count, but without RAII:
- the target embeds `ref_count: std.atomic.Value(u32)` and provides `addRef()` / `release()`
  with the same memory orders as Jolt (`.monotonic` for add, `.release`/`.acq_rel` for release);
- a `Ref<T>` member becomes `*T` (or `?*T`), and the owning struct calls `addRef()` when storing
  it and `release()` in `deinit`. Document ownership in the field's doc comment;
- `RefConst<T>` becomes `*const T`; `release` on a const pointer uses `@constCast` internally.

### Strings, I/O and logging

- `Trace(fmt, ...)` → `std.log.scoped(.zolt).info/warn(...)`; applications control output via
  `std_options.logFn`.
- `StreamIn` / `StreamOut` are ported as interfaces over `*std.Io.Reader` / `*std.Io.Writer`.

## 6. Polymorphism (virtual functions)

Use one of two patterns. Both keep Jolt's ability to add user-defined subclasses.

**A. Class hierarchies with data in the base class** (`Shape`, `ConvexShape`, `Constraint`,
`ShapeSettings`, `BroadPhase`, ...): the base struct holds a `vtable: *const VTable` plus its
data; each derived struct embeds its parent as the first field named `base`; downcasting uses
`@fieldParentPtr`.
```zig
pub const Shape = struct {
    pub const VTable = struct {
        getLocalBounds: *const fn (self: *const Shape) AABox,
        destroy: *const fn (self: *Shape) void,
        // ... one entry per C++ virtual function, in declaration order
    };
    vtable: *const VTable,
    ref_count: std.atomic.Value(u32) = .init(0),
    user_data: u64 = 0,
    shape_type: ShapeType,
    sub_type: ShapeSubType,

    pub fn getLocalBounds(self: *const Shape) AABox {
        return self.vtable.getLocalBounds(self);
    }
};

pub const BoxShape = struct {
    base: ConvexShape, // ConvexShape in turn has `base: Shape`
    half_extent: Vec3,
    convex_radius: f32,

    const vtable: Shape.VTable = .{ .getLocalBounds = getLocalBoundsImpl, ... };

    fn getLocalBoundsImpl(shape: *const Shape) AABox {
        const self: *const BoxShape = @alignCast(@fieldParentPtr("base", @as(*const ConvexShape, @alignCast(@fieldParentPtr("base", shape)))));
        return .{ .min = self.half_extent.negate(), .max = self.half_extent };
    }
};
```
Virtuals with a default implementation in the base: the base exposes the default as a `pub fn`
that derived vtables can reference. Jolt's switch-on-subtype dispatch tables (e.g.
`CollisionDispatch`) are ported as tables, exactly like the C++.

**B. Pure interfaces / listeners** (`ContactListener`, `BodyActivationListener`,
`BroadPhaseLayerInterface`, `ObjectLayerPairFilter`, `JobSystem`, ...): a type-erased fat pointer
like `std.mem.Allocator`:
```zig
pub const ContactListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,
    pub const VTable = struct {
        onContactValidate: ?*const fn (ptr: *anyopaque, ...) ValidateResult = null, // null = Jolt's default implementation
        onContactAdded: ?*const fn (ptr: *anyopaque, ...) void = null,
    };
    /// Build a ContactListener from any `*T` that declares some of the methods (comptime-generated vtable)
    pub fn init(impl: anytype) ContactListener { ... }
};
```
Optional virtuals (with an empty/default C++ body) are nullable entries so implementations only
write the callbacks they need.

**Visitors / templated callbacks** (`template <class Visitor> void Walk(Visitor &)`) become
`anytype` parameters, which keeps static dispatch like the C++.

## 7. Templates and macros

| C++                                         | Zig                                                         |
|---------------------------------------------|-------------------------------------------------------------|
| `template <class T> class Foo`              | `pub fn Foo(comptime T: type) type { return struct {...}; }` |
| `template <int N> void f()`                 | `fn f(comptime n: i32) void` (or `comptime_int`)            |
| `Swizzle<SWIZZLE_Y, SWIZZLE_X, ...>()`      | `.swizzle(.y, .x, ...)` (comptime enum parameters)          |
| `JPH_ASSERT(x)` / `JPH_ASSERT(x, "msg")`    | `std.debug.assert(x)` (keep the message as a comment)       |
| `JPH_IF_ENABLE_ASSERTS(x)`                  | `if (Core.enable_asserts) { x }`                            |
| `JPH_IF_DEBUG(x)` / `#ifdef JPH_DEBUG`      | `if (builtin.mode == .Debug)`                               |
| `#ifdef JPH_DOUBLE_PRECISION`               | `if (Core.double_precision)` (comptime known)               |
| `#ifdef JPH_DEBUG_RENDERER`                 | not ported yet: leave `// TODO(debug_renderer): ...` where code is skipped |
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
3. **Negation is `0 - x`**, which maps -0 to +0 (`Vec3.negate()` does this).
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

## 9. Threading

Zig 0.16 moved blocking synchronization into the `std.Io` interface:
- `std::mutex` → `std.Io.Mutex` (`lock(io)` / `unlock(io)`); `std::condition_variable` →
  `std.Io.Condition`; `Semaphore` → `std.Io.Semaphore`. Types that lock therefore need an
  `io: std.Io`, passed at `init` (like an allocator) and stored.
- `std::atomic<T>` → `std.atomic.Value(T)`; memory orders: `relaxed` → `.monotonic`,
  `acquire`/`release`/`acq_rel`/`seq_cst` map 1:1.
- `std::thread` → `std.Thread.spawn`; cache line padding → `std.atomic.cache_line`.
- `JobSystem` is interface pattern B; `JobSystemThreadPool` is built on `std.Thread` + `std.Io`
  primitives. Multithreaded results must be identical to single threaded ones (Jolt guarantees
  this, so the port must keep the same barriers and sorting of results).

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
- Debug builds on x86_64 use the self-hosted backend. If you suspect a codegen issue
  (especially with `@Vector`), compare with `zig build test -Duse_llvm=true`.
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
| `FLT_MIN`, `FLT_MAX`, `FLT_EPSILON`| `math.flt_min`, `math.flt_max`, `math.flt_epsilon` |                             |
