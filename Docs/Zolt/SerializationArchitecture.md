# Serialization architecture (Phase 8)

This is the binding design for Phase 8:
- `Jolt/Core/RTTI`, `Jolt/Core/Factory` and `Jolt/Core/StreamUtils`;
- `Jolt/ObjectStream` (text and binary streams);
- the serializable declarations of every class outside SoftBody and Hair (`JPH_DECLARE_SERIALIZABLE_*` /
  `JPH_IMPLEMENT_SERIALIZABLE_*`; Phase 9 declared the rest, S17);
- `UnitTests/ObjectStream/ObjectStreamTest.cpp`;
- milestone M9.

It adds to [PortingGuide.md](PortingGuide.md), [CollisionArchitecture.md](CollisionArchitecture.md),
[SimulationArchitecture.md](SimulationArchitecture.md) and [CharacterVehicleArchitecture.md](CharacterVehicleArchitecture.md);
read those first. Every decision below is an instruction for porters. A porter who finds a decision wrong reports it with
the evidence instead of working around it.

The design was chosen from three competing prototypes (fidelity, Zig design, delivery), two judges and a synthesis. The
synthesis prototype became the foundation (section 10): the RTTI, the Factory, the streams, two real classes and the
port of ObjectStreamTest. It wrote Jolt's exact bytes and read them back.

Phase 8 is done. The groups of section 4 are merged, all 81 classes are declared and milestone M9 is reached (section
10). Section 12 collects what the groups and the final audit settled on the way.

## 1. Scope

| Group | Jolt files | Tests | Lines (C++, `wc -l`) |
|---|---|---|---|
| RTTI and Factory | `Core/RTTI`, `Core/Factory` | (none) | 731 |
| StreamUtils | `Core/StreamUtils.h` | (none) | 163 |
| ObjectStream core | `ObjectStream/ObjectStream`, `ObjectStreamTypes`, `SerializableAttribute`, `SerializableAttributeTyped`, `SerializableAttributeEnum`, `GetPrimitiveTypeOfType`, `SerializableObject`, `TypeDeclarations` | `UnitTests/ObjectStream/ObjectStreamTest.cpp` | 993 |
| Writers | `ObjectStream/ObjectStreamOut`, `ObjectStreamTextOut`, `ObjectStreamBinaryOut` | (same) | 806 |
| Readers | `ObjectStream/ObjectStreamIn`, `ObjectStreamTextIn`, `ObjectStreamBinaryIn` | (same) | 1565 |
| Declarations | the `sCreateRTTI` bodies of 81 classes in 64 `.cpp` files (Hair and SoftBody excluded) | (none) | (part of their files) |
| M9 | `PerformanceTest/RagdollScene.h` | (none) | 144 |

- The lines are `wc -l` of the `.h`, `.inl` and `.cpp` files in the "Jolt files" column. The test (257 lines) is not
  counted.
- The 81 declarations: 6 TypeDeclarations classes, SerializableObject, LinearCurve (2), bodies and collision (7), the
  scene (2), shapes (22), constraints (19), vehicles (13), skeleton and ragdoll (9).
- Zolt declares them in 64 Zig files.
- Before Phase 8 the declarations were `// TODO(serialization)` comments: 111 lines in 65 Zolt files. One line was left,
  in `SoftBodyCreationSettings.zig`. Phase 9 replaced it with the declaration (S17).

**Milestone M9.** ZoltPerformanceTest runs `PerformanceTest/RagdollScene.h`. It reproduces the hashes of
`determinism_check.yml` with `-q=LinearCast -t=max -s=Ragdoll`:
- `0x3be6ea76c07a2ac8` in single precision;
- `0x8dd6bc501859a228` in double precision.

The hashes must also hold with `-t=1` and `-t=2`, and Zolt's CI checks them. M9 is reached: both hashes hold with
`-t=max`, `-t=1` and `-t=2` (section 10). The scene loads three assets through ObjectStreamIn:
- `Assets/Human.tof` (text, a RagdollSettings);
- `Assets/Human/dead_pose1.tof` (text, a SkeletalAnimation);
- `Assets/terrain2.bof` (binary, a PhysicsScene).

All three are written with an older schema. M9 therefore depends on Jolt's reading rules: aliases, skipped attributes of
another type, defaults of missing attributes, and the `vec3` to `DVec3` conversion in double precision.

## 2. Decisions

### S1 Reference build

- The C++ reference of `zig build parity` is built with `-DJPH_OBJECT_STREAM`, Jolt's CMake default
  (`ENABLE_OBJECT_STREAM`). Every C++ translation unit gets it: Jolt, the parity references, HelloWorld and
  PerformanceTest. The define changes the RTTI layout and `JPH_VERSION_ID`, so they must agree.
- The parity tests run with the repository root as working directory, so that they can read `Assets/`.
- New step `zig build jolt-performance-test -- <args>`: the C++ PerformanceTest linked against the parity reference
  library. It is the M9 oracle. A prototype showed that it gives both published Ragdoll hashes, so the reference toolchain
  (zig c++, libc++, glibc) is exact for M9. Zolt's CI runs it in both precisions, before the Ragdoll steps of Zolt.
- Consequences of the define:
  - the C++ Factory also registers the attribute classes: 104 classes in single precision, 105 in double precision;
  - Jolt's configuration string gains "(ObjectStream) "; Zolt's `ConfigurationString.zig` adds it unconditionally;
  - nothing else changes. The fidelity prototype ran the complete parity suite with the define: 631 passed, 5 skipped.
- Zolt has no build option for ObjectStream. It is always on, like Jolt's default. Unused code costs nothing in Zig.

### S2 RTTI: one compile time constant per class

- `Zolt/Core/RTTI.zig` holds `RTTI`. It has Jolt's data:
  - `name`: Jolt's name, the stringified macro argument. A nested class keeps `::`, for example
    `"RagdollSettings::Part"`. The name is the stream class name, the Factory key and the hash input.
  - `hash`: `RTTI::GetHash`, computed at compile time (`RTTI.hashName`).
  - `kind`, `bases` (with offsets), `attributes` (flattened), `create`, `destroy` and `ref_count`.
- A class declares it next to its fields: `pub const rtti = RTTI.declare(@This(), .{ ... })`. `RTTI.of(T)` is
  `JPH_RTTI(T)`.
- The identity of an RTTI is its address, like Jolt's `operator ==`. A factory cannot hold two RTTIs with one name, so no
  two RTTIs have the same contents.
- `declare` checks the name against `@typeName(T)` (`::` is `.`). `.renamed = true` allows another name
  (JPH_RENAME_CLASS, unused in Jolt).
- The kind is Jolt's IMPLEMENT macro, not the DECLARE macro:

  | Jolt IMPLEMENT macro | Zolt kind | Creatable |
  |---|---|---|
  | NON_VIRTUAL, OUTSIDE_CLASS | `.non_virtual` | yes |
  | VIRTUAL, VIRTUAL_BASE | `.virtual` | yes |
  | ABSTRACT, ABSTRACT_BASE | `.abstract` | no |

- So PhysicsMaterial, ConstraintSettings and WheelSettings are `.virtual`. DecoratedShapeSettings is `.abstract` (Jolt
  declares it VIRTUAL and implements it ABSTRACT).
- A `.virtual` or `.abstract` class with a vtable reads its dynamic type from the vtable (S3). A `.non_virtual` class
  must not have a vtable. A `.virtual` class must have one.
- Zolt's pattern A has one virtual chain (`base`). A second base class has no vtable in Zolt, so it is declared
  `.non_virtual` (ObjectStreamTest's TestSerializableBase2 is the only case).

### S3 The dynamic type: an `rtti` vtable entry

- The root vtables of the 7 serializable pattern A hierarchies get the data entry `rtti: *const RTTI`: ShapeSettings,
  PhysicsMaterial, GroupFilter, ConstraintSettings, PathConstraintPath, VehicleControllerSettings and WheelSettings.
- It replaces the data entry `rtti_name` of 5 roots and `WheelSettings.RTTI`. `getRTTIHash()` becomes
  `self.vtable.rtti.hash`.
- `virtual.make` already fills data entries from `@field(T, name)`. So every concrete class of these hierarchies must
  declare `pub const rtti`, or it fails to compile ("X must declare `pub const rtti`").
- `RTTI.getRTTI(T, ptr)` is Jolt's `GetRTTI`: the vtable entry of the root for virtual classes, the static RTTI for the
  others.
- A user class that has no RTTI of its own in Jolt (for example FrictionPerTriangleTest's material) declares the RTTI
  of its parent: `pub const rtti = PhysicsMaterialSimple.rtti;`. It is then written and created as the parent, exactly
  like Jolt.
  - Such a class is written from its parent's sub object, which need not be at offset 0. `RTTI.mostDerived` and by
    value members start there (`RTTI.RTTIOwner`). This is the address that C++ uses: it converts the object to
    `Parent &`.
  - Nothing tells an object of such a class from an object of its parent. So these are compile errors:
    `RTTI.dynamicCast` to the class, `ObjectStreamIn.sReadObject` of it, and a Ref or a pointer member to it. The
    message is "X uses the RTTI of P, so its objects cannot be told from those of P (use P)". Jolt returns or stores an
    object of the parent as X.

### S4 Bases, offsets and casts

- `.bases = &.{ B1, B2 }` lists Jolt's `JPH_ADD_BASE_CLASS` calls in order. A base is one of:
  - the unique field of type B (`base` of pattern A, or `base2` of a second base);
  - a flattened prefix at offset 0: T starts with B's fields at B's offsets. IndexedTriangle and the empty
    SerializableObject use this.
- The offsets are the real Zig offsets. Zig may reorder fields, so `base` need not be at offset 0.
- `rtti.isKindOf(other)` and `rtti.castTo(object, other)` search the bases depth first in Jolt's order and add the
  offsets.
- `RTTI.mostDerived(T, ptr)` is the address of the most derived object. `RTTI.dynamicCast(To, From, ptr)` is Jolt's
  `DynamicCast`. `StaticCast` stays `virtual.upcast` / `virtual.downcast`.
- Attributes of the bases are copied first, at the base offset, like Jolt's `AddBaseClass`.

### S5 Attribute declarations

| Jolt | Zolt |
|---|---|
| `JPH_ADD_ATTRIBUTE(C, mFoo)` | `.attr("mFoo", .foo)` |
| `JPH_ADD_ATTRIBUTE_WITH_ALIAS(C, mA.mB, "mX")` | `.alias("mX", .{ .a, .b })` |
| `JPH_ADD_ENUM_ATTRIBUTE(C, mFoo)` | `.enumAttr("mFoo", .foo)` |
| `JPH_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(C, mA.mB, "mX")` | `.enumAlias("mX", .{ .a, .b })` |
| a `Ref<Derived>` that Zolt holds as `Ref(Base)` | `.attrAs("mFoo", .foo, Derived)` |
| a member that Zolt stores in another form | `.custom("mFoo", Accessor)` |

- The first argument is the stream name, copied verbatim from Jolt. The second is a field or a path of fields (enum
  literals).
- The order is Jolt's order. A wrong order changes the bytes.
- `.attrAs` sites: `RagdollSettings::Part.mToParent`, `RagdollSettings::AdditionalConstraint.mConstraint` and
  `PhysicsScene::ConnectedConstraint.mSettings`. The stream declares Jolt's class (`TwoBodyConstraintSettings`). A link
  accepts only objects of that class.
- `.custom` site: `BodyCreationSettings.mShape`, which Zolt keeps in the union `shape: ShapeSource`. The accessor
  declares `Member` (`Ref(ShapeSettings)`), `get` (what is written) and `set` (called at link resolution). A null
  identifier calls nothing, so the union keeps `.none`, like Jolt's untouched member.
- These are compile errors, each with a readable message:
  - the class name is not the Zig type name (without `.renamed`);
  - the kind does not fit the class (vtable or not, the `rtti` entry);
  - a creatable class that owns an allocator has no `createDefault`;
  - a base is neither a field nor a flattened prefix;
  - an attribute names a missing member;
  - `.attr` names a member whose name is not the Jolt name (`mHalfExtent` is `half_extent`, `r` is `r`);
  - `.attr` on an enum or a packed struct;
  - an enum whose tag type is not 8, 16 or 32 bits wide;
  - a String member whose default is not `""`;
  - `.attrAs` with a class that does not derive from the member's class;
  - `.attrAs` on a primitive, a String or an array of them ("`.attrAs` is only for references and pointers");
  - a pointer to a primitive or a String, for example `?*f32` ("a pointer to f32 has no stream type");
  - a Ref or a pointer to a class that uses the RTTI of its parent (S3);
  - more than 32 attributes or more than 4 bases (Jolt's StaticArrays);
  - one attribute name twice, also between a base and the class;
  - two attributes that share memory;
  - a member type without a stream type;
  - two classes with one name or one hash in a factory.
- Zig cannot test that code does not compile, so these checks were verified by hand. The header of
  `Zolt/ObjectStream/ObjectStream.zig` lists the messages of the `.attrAs`, pointer and Ref checks.
- An integer that Jolt declares with JPH_ADD_ENUM_ATTRIBUTE (`mObjectLayer`) needs `.enumAttr`. `.attr` on it is no
  compile error, because the integer type cannot tell. `tools/gen_serializable.py --check` catches it for Jolt's
  classes, and P1 compares the declared types. A user class must be checked by hand (section 7).

### S6 Member types

| Zig member | Jolt | Declared as | Data |
|---|---|---|---|
| `u8 u16 i32 u32 u64 f32 f64 bool Float3 Float4 Double3 Vec3 DVec3 Vec4 UVec4 Quat Mat44 DMat44` | primitive | `uint8` ... `dmat44` | the value |
| `RVec3`, `Real` | primitive | `vec3` / `dvec3`, `float` / `double` by precision | the value |
| `[]u8` / `[]const u8` | `String` | `string` | escaped text / string table |
| a struct with an RTTI | a class by value | `instance C` | its attributes inline |
| `Ref(C)` / `RefConst(C)` | `Ref<C>` / `RefConst<C>` | `instance C` (Jolt's quirk) | an identifier |
| `?*C` / `?*const C` | `C *` | `pointer C` | an identifier |
| `std.ArrayList(E)` | `Array<E>` | `array` + E | a count, then the elements |
| `StaticArray(E, N)` | `StaticArray<E, N>` | `array` + E | the size; reading fails above N |
| `[N]E` | `E[N]` | `array` + E | always N; reading fails for another count |
| an enum, a packed struct, an integer (`.enumAttr`) | JPH_ADD_ENUM_ATTRIBUTE | `uint32` | `uint32(value)` |

- The codecs are generated at compile time from the member type (`ObjectStream.Codec`). They are Jolt's OSIsType,
  OSReadData, OSWriteData and OSWriteDataType overloads.
- `getMemberPrimitiveType` is a plain pointer to the element's RTTI. Zig resolves pointers to constants lazily, so classes
  that refer to each other (or to themselves) compile.
- A String member owns its bytes. The reader frees the old string with the stream's allocator.

### S7 Creating and destroying objects

- `rtti.create(allocator)` is Jolt's `new T`:
  - a class that owns an allocator (pattern A, RefTargets) calls `T.createDefault(allocator) Allocator.Error!*T`. It is
    mandatory for every creatable class of this kind (a compile error otherwise). Shape settings add it
    (`self.* = .initDefault(allocator)`). Skeleton, SkeletalAnimation, RagdollSettings, PhysicsScene and the
    WheelSettings use `pub const createDefault = create;`.
  - a value class uses `Lifetime.defaultValue`: the field defaults, or `T.initDefault(allocator)` when the default
    allocates (VehicleEngineSettings, VehicleTransmissionSettings).
- `rtti.destroy` is Jolt's `delete`: the generated `vtable.destroy`, or `deinit()` and a free with the object's own
  allocator, or `Lifetime.destruct` and a free.
  - An abstract class with a vtable has `destroy` too. `destructObject` deletes it through the virtual destructor, like
    Jolt.
  - Only SerializableObject has no `destroy`, because Zolt's SerializableObject has no vtable. `destructObject` panics
    for it, where Jolt deletes through the virtual destructor.
  - `isAbstract()` is Jolt's `create == null or destroy == null`.
- New objects have reference count 0, like `new T`.
- `Core/Lifetime.zig` (a Zolt addition) holds `defaultValue`, `defaultConstruct`, `destruct` and `hasDestructor`.
  Core/StreamIn.zig switches from its private copies to it, so the binary state and the object streams share one
  definition.
- A member that C++ leaves indeterminate is zero (D9).

### S8 Factory

- `Zolt/Core/Factory.zig` holds `Factory`, a value built at compile time: `Factory.init(&.{ T, ... })`.
- `init` replays Jolt's `Register`: the class, then its bases, then the classes of its attributes, each once, in
  pre-order.
- `find(name)`, `findHash(hash)` and `createObject(allocator, name)` are Jolt's. A name ends at the first NUL, like
  Jolt's `const char *` key. `getAllClasses()` returns the registration order; Jolt returns its hash map order, and
  nothing depends on it.
- There is no global, no `sInstance`, no run time `Register` and no `Clear`.
- `RegisterTypes.zig` gets:
  - `factory_types`: the types of `RegisterTypes.cpp`, in that order. Jolt has 49. Zolt had 48 in Phase 8, without
    SoftBodyCreationSettings; Phase 9 added it as the 49th (S17);
  - `factory = Factory.init(factory_types ++ user_types.serializable_types)`.
- Tests and applications build their own factory when they need other classes:
  `Factory.init(RegisterTypes.factory_types ++ &[_]type{MyClass})`. This is Jolt's
  `Factory::sInstance->Register(JPH_RTTI(MyClass))`.

### S9 Restoring binary state by hash

- `Core/StreamUtils.zig` ports `StreamUtils.h`. It moves the object reference helpers out of `Shape.zig`.
- `restoreObject(Root, ...)` reads the hash and looks it up with `RegisterTypes.factory.findHash`. The class must be
  creatable and `isKindOf(Root.rtti)`; otherwise the result is Jolt's error "Failed to create instance of type".
- It replaces the five comptime lists (`material_types`, `group_filter_types`, `constraint_settings_types`,
  `path_constraint_path_types`, `vehicle_controller_settings_types`) and the five copies of `rttiHash`.
- Results are Jolt's for every hash on which Jolt is defined. A hash of another hierarchy or of an abstract class is
  undefined in Jolt; Zolt reports the error (D8).
- `PhysicsScene.restoreFromBinaryState` reports the same error for constraint settings that are not a
  TwoBodyConstraintSettings, such as ConstraintSettings itself. Jolt does an unchecked StaticCast there (D8).
- ConstraintSettings becomes creatable (`.virtual`, `createDefault`), like Jolt.
- User classes register through `zolt_user_types.serializable_types`. A user module that still declares one of the five
  old list names gets a compile error that names the new hook.

### S10 Stream API

```zig
// ObjectStreamOut.zig (sWriteObject): `object` is written as its dynamic class; the caller flushes the writer
pub fn sWriteObject(comptime T: type, allocator: Allocator, writer: *std.Io.Writer, stream_type: StreamType, object: *const T) Error!void;

// ObjectStreamIn.zig (sReadObject): the main object with the reference count that the stream's links gave it
pub fn sReadObject(comptime T: type, allocator: Allocator, reader: *std.Io.Reader, options: Options) Error!*T;
pub fn sReadObjectRef(comptime T: type, allocator: Allocator, reader: *std.Io.Reader, options: Options) Error!Ref(T);
pub const Options = struct { factory: *const Factory = &RegisterTypes.factory };
```

- The file variants `sReadObjectFromFile`, `sReadObjectRefFromFile` and `sWriteObjectToFile` take `io: std.Io` and a
  path. They read and write through a buffered `std.Io.File` reader or writer.
- Text and binary are a closed set of final classes, so `ObjectStreamIn` and `ObjectStreamOut` hold a tagged union
  `format` (guide section 6). `IObjectStreamIn` and `IObjectStreamOut` disappear: codecs take the concrete streams.
- Errors:
  - reading: `error{ OutOfMemory, ReadFailed, InvalidObjectStream }`. `InvalidObjectStream` is every Jolt `false`.
    `ReadFailed` is an I/O error of the reader (D13).
  - writing: `Allocator.Error || std.Io.Writer.Error`.
- Jolt's Trace texts go to `std.log.scoped(.zolt).info`.
- A value class read as the main object is freed with `RTTI.of(T).destructObject(p, allocator)`. A RefTarget is wrapped
  in a Ref (`sReadObjectRef`).
- This free is always safe. When T has no vtable, `sReadObject(T)` accepts only a main object of exactly class T (D17).
  When T has one, `destructObject` deletes through the virtual destructor.

### S11 Reading and writing follow Jolt statement by statement

`ObjectStreamIn.zig` and `ObjectStreamOut.zig` are the reference. In short:

- **Writer.**
  - The header is `TOS 1.00` or `BOS 1.00`.
  - The main object gets identifier 1. Other objects get the next identifier when a reference to them is first written.
    They are written in that order (a FIFO queue).
  - `write` keeps the identifier and class of an object that already has one (Jolt's `try_emplace`). The counter still
    goes up, so a second `write` on one stream skips an identifier, like Jolt.
  - A class is declared once, right before the first object that needs it. Its attribute classes are queued, also
    abstract ones. Bases and primitives are never declared.
  - Objects are identified by the address of the most derived object (S4).
  - Text: `\r\n` and two spaces per indentation level, `%08X` identifiers, floats `%.9g`, doubles `%.17g`
    (`StringTools.writeFloatPrecision`), Jolt's string escapes.
  - Binary: little endian, Vec3 as 12 bytes, DMat44 as 72 bytes, one string table for names and values. A string
    length is Jolt's `min((uint32)size, 0x7fffffff)`: truncated to 32 bits first, then capped.
- **Reader.**
  - The main loop reads `declare` and `object` tokens. It ends when no data type can be read: at the end of the
    stream, or on an unknown word in a text stream.
  - Attributes match by name (the first match in the flattened list). The type must match exactly; `vec3` and `dvec3`
    convert into each other. Unmatched attributes are skipped. Missing attributes keep their defaults.
  - Class names in text are whole words. Where Jolt looks a name up as a `const char *`, the name ends at the first
    NUL: in the factory (`find`, `createObject`), for the class of an attribute (`IsType`, a `strcmp`) and for the
    class of a skipped instance. The key of a class description and the class of an `object` use the whole word, like
    Jolt.
  - The first object that is created is the main object. It must be kind of the requested class. `sReadObject` also
    requires exactly that class when the class has no vtable (D17).
  - An object whose data cannot be read is destroyed, and the read goes on.
  - Links resolve after the whole stream, in read order.
  - Objects that no link references are destroyed, except the main object.
- **Ownership.**
  - One allocator per read. Every object, string and array of the read comes from it. Objects that keep an allocator
    keep this one.
  - Arrays are sized once before their elements are read. So the member addresses that links store stay valid until
    the links resolve.
  - A link stores the member, the identifier, Jolt's declared class (the IsKindOf check) and a store callback. The
    callback casts to the member's class (`castTo`) and calls `Ref.set` (one more reference), or stores a raw pointer.
  - Reference counts are Jolt's: one per Ref holder; raw pointers do not count. The main object has the count of its
    incoming links, usually 0.

### S12 Text numbers follow the reference platform

- Jolt reads a number word with `IStringStream >> value`. The result depends on the C++ standard library. Zolt follows
  the parity reference: libc++ with glibc's `strtof` / `strtod`.
- Integers: an optional sign and digits; the rest of the word is ignored (`12abc` is 12). An unsigned value is negated
  modulo 2^n (`-1` is 4294967295). Out of range fails. uint8 and uint16 are read as uint32 and truncated.
- Floats: num_get collects the characters `0-9a-fA-FxX+-pP` and one `.`; they must form a whole decimal or hexadecimal
  number (`1.5abc` fails, `1.2.3` is 1.2). The value is rounded once, directly to f32 or f64. Overflow, a subnormal
  result and a nonzero number that rounds to zero fail (ERANGE).
- glibc detects tininess after rounding. It rounds the exact value to T's precision with an unbounded exponent, then
  compares the result with the smallest normal number.
  - So a word fails when its exact value is below the smallest normal number by more than half an ulp at that
    precision. For float the ulp just below FLT_MIN is 2^-150, so the limit is FLT_MIN - 2^-151. For double the limit
    is DBL_MIN - 2^-1076.
  - Such a word fails even when it rounds to the smallest normal number: `1.1754943e-38` fails. `1.17549435e-38` is
    within the limit, so it is read as FLT_MIN.
  - A subnormal float in a text stream cannot be read back, in Jolt too. The writers never write the exact value of a
    subnormal, which glibc would read (D14).
- Identifiers are `strtoul(word, 16)` truncated to 32 bits.
- `ObjectStreamTextIn.zig` implements these rules. Parity test P6 pins them.

### S13 Enums

- An enum attribute is written as `uint32(value)`; a signed tag is sign extended.
- It is read as `static_cast`: the uint32 is truncated to the tag type.
- The tag type is Jolt's underlying type. `MeshShapeSettings.BuildQuality` becomes `enum(i32)` (Jolt: `int`).
- The guide's rule for enums read from a stream applies (non-exhaustive, Jolt's width). ESpringMode, ESwingType,
  ETransmissionMode, EConstraintSpace and EPathRotationConstraintType already follow it. BuildQuality follows it too.
- MotionType, MotionQuality and OverrideMassProperties stay exhaustive, like the binary state of BodyCreationSettings.
  An unnamed value keeps the member's value, logs a trace and the read goes on (D3).
- `AllowedDOFs` (a packed struct) takes all 8 bits. `ObjectLayer` (an integer) is truncated. An integer needs
  `.enumAttr` by hand (S5).

### S14 Precision

- `RVec3` attributes are `vec3` in single precision and `dvec3` in double precision. The stream schema therefore
  differs between the precisions.
- The reader converts `vec3` to DVec3 (`DVec3.fromVec3`) and `dvec3` to Vec3 (`toVec3`, round to nearest). M9 in double
  precision uses the first conversion for every position in the assets.
- Every parity layer runs in both precisions.

### S15 Jolt's quirks are reproduced

- A Ref is declared `instance X` but its data is an identifier. Skipping a Ref attribute (unknown name, wrong type)
  reads class data instead, so the stream is misread.
- A null identifier leaves the member untouched.
- A second `declare` of a class appends its attributes to the first.
- An object of an undeclared class is not skipped. Its identifier is read as the next data type.
  - In text that word is no data type, so the read ends.
  - In binary, identifier 0 or 1 reads on as `declare` or `object` (a misread). Any other value is a fatal "out of place
    token", which also destroys the main object.
- The failure of a skipped object is ignored.
- Static attribute classes are declared even when they are abstract or unused.
- WheelSettings has no RTTI base, so it is not kind of SerializableObject.
- Binary strings are cut at the first NUL. The empty string never enters the string table. The writers write the
  whole string, so a string with a NUL does not survive a binary round trip, in Jolt either.
- A C array needs exactly N elements; a StaticArray at most N.

### S16 Zolt's defined behavior where Jolt is undefined

None of these cases occurs in a stream that Jolt or Zolt writes. D8 is about a binary state, D10 about the allocator
and D17 about the class that the caller requests.

| # | Input | Jolt | Zolt |
|---|---|---|---|
| D1 | an object's data fails after links into it were recorded | resolves them into freed memory | drops them with the object |
| D2 | a second object with a used identifier | leaks it; its links still resolve | destroys it with its links |
| D3 | an unnamed value of an exhaustive enum | stores the invalid value | keeps the member, logs a trace |
| D4 | only an unreferenced object references the main object | deletes the main object, returns a dangling pointer | keeps the main object alive (a temporary reference) |
| D5 | a binary bool byte other than 0 or 1 | undefined | true |
| D6 | a binary string length beyond the stream | `alloca(len)`, a stack overflow | reads in chunks, fails at the end |
| D7 | a text identifier after a stale ERANGE | may fail | only a 64 bit overflow fails |
| D8 | a binary state hash of another hierarchy or of an abstract class; PhysicsScene constraint settings that are not a TwoBodyConstraintSettings (ConstraintSettings itself, VehicleConstraintSettings) | a wrong `static_cast` or a null dereference; an unchecked StaticCast | "Failed to create instance of type" |
| D9 | members that C++ leaves indeterminate (`new Color`) | garbage | zero |
| D10 | allocation failure | abort | `error.OutOfMemory`, nothing leaked |
| D11 | two stream attributes that map to one attribute of a class | reads the member twice: dangling links into a reallocated array, leaked references | `InvalidObjectStream` |
| D12 | Ref / RefConst links that form a cycle | reads the objects and leaks them | `InvalidObjectStream` |
| D13 | an I/O error of the reader | ends the read like the end of the stream | `error.ReadFailed` |
| D14 | a float word whose exact value is a subnormal of T (hexadecimal, or decimal with all its digits) | glibc reads it without ERANGE | fails like every subnormal |
| D15 | class data nested deeper than `max_nesting_depth` (a class that contains itself by value in a crafted stream) | recurses until the stack overflows | `InvalidObjectStream` |
| D16 | a MeshShapeSettings build quality without a name (S13) | `JPH_ASSERT(false)`, then a call through a null splitter | the value is read, kept and written back; creating the shape panics with a message |
| D17 | `sReadObject(T)` where T has no vtable, and a main object of a derived class (RagdollSettings::Part read as BodyCreationSettings, IndexedTriangle as IndexedTriangleNoMaterial) | returns the derived object, which the caller deletes through a `T *`: the wrong size, the derived part leaks | logs Jolt's "Main object of different type", releases the object (unless a Ref of another object holds it) and returns `InvalidObjectStream` |

- D11: no Jolt class has two attributes in one place, and a writer declares each attribute once.
- D12: a Ref cycle is useless for Jolt's classes; a compound shape that contains itself recurses forever in CreateShape.
- D14: accepting an exact subnormal word needs arbitrary precision. The writers never produce one (S12).
- D17: `ObjectStreamIn.read` is unchanged and returns the dynamic class. Only `sReadObject` checks it, because its
  caller frees the object as a T (S10).
- The parity generators never produce these inputs. Zolt-only tests pin them:
  - `ZoltTests/ObjectStream/ObjectStreamZoltTests.zig` pins D1 to D7, D9 and D11 to D17;
  - D8 is pinned by the inline tests of `Core/StreamUtils.zig` and `Physics/PhysicsScene.zig`, D10 by the sweep Z2;
  - D16 is pinned on the read side only: the panic cannot be tested in process.

### S17 Phase 9 boundary

- SoftBodyCreationSettings, SoftBodySharedSettings (with its 11 nested classes) and the 10 Hair classes stay in Phase 9.
  So do the 7 ComputeSystem RTTIs and `RegisterHair`.
- PhysicsScene declares `mBodies` and `mConstraints`, but not `mSoftBodies`, until Phase 9.
- Reading a scene without soft bodies (terrain2.bof) is exact: an empty `mSoftBodies` array skips nothing. A stream with
  soft bodies fails to read, because skipping their `mSettings` Ref misreads the stream (S15).
- Writing a PhysicsScene lacks Jolt's `mSoftBodies` and the soft body declarations. Its parity is a cross-read: the C++
  side reads Zolt's stream and must write what it writes for the same scene.
  - A string with a NUL (a PhysicsMaterialSimple debug name in a text stream) is cut when Jolt reads binary bytes
    (S15), its own bytes too. When a string has a NUL, the binary comparison takes the bytes that Jolt writes after
    reading its own binary bytes. Without a NUL, and in text, the bytes must equal Jolt's first write.
- P1 has an explicit exclusion list of the 13 SoftBody classes (`phase9_classes` in `ObjectStreamSchema.zig`).
  `tools/gen_serializable.py` skips the SoftBody and Hair directories and PhysicsScene's "mSoftBodies". Phase 9 removes
  these exclusions.
- Settled in Phase 9 ([RemainingFeaturesArchitecture.md](RemainingFeaturesArchitecture.md) R14, R19): the boundary is
  gone.
  - The 13 SoftBody classes and the 10 Hair classes are declared, and so are `RegisterHair` and the RTTIs of
    ComputeSystem and ComputeSystemCPU. The RTTIs of the GPU compute systems stay deferred with their backends.
  - `RegisterTypes.factory_types` has Jolt's 49 types: SoftBodyCreationSettings is the 49th.
  - PhysicsScene declares `mSoftBodies` as its third attribute. A stream with soft bodies reads like Jolt's.
  - The exclusions are gone: `phase9_classes`, the SoftBody and Hair directories and "mSoftBodies" of
    `gen_serializable.py`, `PHASE9_SERIALIZABLE` of `port_status.py`.
  - P7 compares a PhysicsScene directly, soft bodies included. The cross-read and its NUL rule (section 12) are gone.

### S18 Pending declarations during the phase

- F2 gave every one of the 81 classes a declaration: the name, the kind, the bases and, where needed, `createDefault`.
  A declaration without its attributes carried `pub const serialization_pending = true;`.
- Each group added the attributes and removed the marker. No marker is left.
- During the phase the parity layers skipped a pending class and every class whose attributes reach one.
- This kept the `rtti` vtable entry non-null from the start, and every group depended only on the foundation.
- The final check is permanent:
  - `python3 tools/port_status.py --check` (the lint step of CI) runs `tools/gen_serializable.py --check --complete`. It
    fails on any pending or undeclared class. `--no-pending` remains as a flag of its own. Since Phase 9 it covers 104
    classes: these 81, the 13 SoftBody classes and the 10 Hair classes (S17).
  - P1 fails for a missing or pending class and for a class that reaches a pending one. P2 to P7 fail instead of
    skipping such a class.

## 3. Recipe: the declaration of each class pattern

Put the declaration first in the struct, with a doc comment that names Jolt's macros. Replace the
`// TODO(serialization)` comment and the file header note.

**A value class with an enum** (in the tree, `SpringSettings.zig`):

```zig
const RTTI = @import("../../Core/RTTI.zig").RTTI;

pub const SpringSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SpringSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "SpringSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .enumAttr("mMode", .mode),
            .attr("mFrequency", .frequency),
            .attr("mDamping", .damping),
        },
    });

    mode: SpringMode = .frequency_and_damping,
    ...
```

**Aliases into a nested member** (in the tree, `MotorSettings.zig`):

```zig
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(MotorSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "MotorSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .enumAlias("mSpringMode", .{ .spring_settings, .mode }),
            .alias("mFrequency", .{ .spring_settings, .frequency }), // Renaming attributes to stay compatible with old versions of the library
            .alias("mDamping", .{ .spring_settings, .damping }),
            .attr("mMinForceLimit", .min_force_limit),
            ...
```

**A class declared outside** (in the tree, `ObjectStream/TypeDeclarations.zig`; Geometry and Core types never import
the ObjectStream):

```zig
    .{
        IndexedTriangle,
        RTTI.declare(IndexedTriangle, .{
            .name = "IndexedTriangle",
            .kind = .non_virtual,
            .bases = &.{IndexedTriangleNoMaterial}, // a flattened prefix (extern structs)
            .attributes = &.{ .attr("mMaterialIndex", .material_index), .attr("mUserData", .user_data) },
        }),
    },
```

**A pattern A chain** (root, abstract level, concrete class):

```zig
// Shape.zig: the root gets the vtable entry
pub const ShapeSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT_BASE(ShapeSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "ShapeSettings",
        .kind = .abstract,
        .bases = &.{SerializableObject},
        .attributes = &.{.attr("mUserData", .user_data)},
    });

    pub const VTable = struct {
        /// GetRTTI(): JPH_RTTI of the most derived class (every concrete class declares `rtti`)
        rtti: *const RTTI,
        deinit: ..., destroy: ..., createShape: ...,
    };
    ...

// ConvexShape.zig: an abstract level
pub const ConvexShapeSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(ConvexShapeSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "ConvexShapeSettings",
        .kind = .abstract,
        .bases = &.{ShapeSettings},
        .attributes = &.{ .attr("mDensity", .density), .attr("mMaterial", .material) },
    });
    ...

// SphereShape.zig: a concrete class
pub const SphereShapeSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SphereShapeSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "SphereShapeSettings",
        .kind = .virtual,
        .bases = &.{ConvexShapeSettings},
        .attributes = &.{.attr("mRadius", .radius)},
    });
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    radius: f32 = 0.0,

    /// new SphereShapeSettings (RTTI create): reference count 0, the last release() destroys it
    pub fn createDefault(allocator: Allocator) Allocator.Error!*SphereShapeSettings {
        const self = try allocator.create(SphereShapeSettings);
        self.* = .initDefault(allocator);
        return self;
    }
    ...
```

**A RefTarget without a vtable, with Refs** (RagdollSettings):

```zig
pub const RagdollSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(RagdollSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "RagdollSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .attr("mSkeleton", .skeleton), // Ref(Skeleton): `instance Skeleton`, an identifier
            .attr("mParts", .parts), // `array instance RagdollSettings::Part`, inline
            .attr("mAdditionalConstraints", .additional_constraints),
        },
    });

    pub const Part = struct {
        /// JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(RagdollSettings::Part)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "RagdollSettings::Part",
            .kind = .non_virtual,
            .bases = &.{BodyCreationSettings},
            .attributes = &.{.attrAs("mToParent", .to_parent, TwoBodyConstraintSettings)}, // Ref(ConstraintSettings) in Zolt
        });
        base: BodyCreationSettings = .{},
        to_parent: Ref(ConstraintSettings) = .empty,
        ...
    };

    /// new RagdollSettings (RTTI create)
    pub const createDefault = create;
    ...
```

**Enum, flag and integer attributes** (BodyCreationSettings, excerpt):

```zig
    .enumAttr("mObjectLayer", .object_layer), // an integer: uint32, truncated when read; .attr would compile (S5)
    .enumAttr("mMotionType", .motion_type), // exhaustive enum(u8): an unnamed value keeps the member (D3)
    .enumAttr("mAllowedDOFs", .allowed_dofs), // packed struct(u8): its 8 bits
    .alias("mSensorDetectsStatic", .collide_kinematic_vs_non_dynamic), // This is the old name to keep backwards compatibility
```

**A custom member** (BodyCreationSettings' `mShape`, the union `shape`):

```zig
    .custom("mShape", ShapeAttribute),

const ShapeAttribute = struct {
    pub const Member = Ref(ShapeSettings); // the stream sees `instance ShapeSettings`
    pub fn get(self: *const BodyCreationSettings) Member { // written; borrowed, no reference added
        return switch (self.shape) {
            .settings => |s| .{ .ptr = s.ptr },
            else => .empty,
        };
    }
    pub fn set(self: *BodyCreationSettings, settings: ?*ShapeSettings) void { // at link resolution
        self.releaseShape();
        if (settings) |s| self.shape = .{ .settings = .init(s) };
    }
};
```

**Two bases and a user class** (in the tree, `ZoltTests/ObjectStream/ObjectStreamTest.zig`): the second base is the field
`base2`; `.bases = &.{ TestSerializableBase, TestSerializableBase2 }`. A user class registers through
`zolt_user_types.serializable_types` or a factory of its own (S8).

**Steps for every class.**
1. Copy the `sCreateRTTI` body of the `.cpp` file: same names, same order, same aliases.
2. Pick the kind from the IMPLEMENT macro (S2).
3. Add `createDefault` if the class is creatable and owns an allocator (S7).
4. Use `.attrAs` / `.custom` only where S5 lists them. Report any other difference of member type to the foundation.
5. Remove `serialization_pending` and the `TODO(serialization)` lines. Update the file header (`Status`).
6. Run P1 and P2 for the class (section 5). Make one attribute wrong on purpose and see them fail.

## 4. Groups and order

All groups are done. F1, F2 and F3, then G1 to G6, are merged. G7 was a completeness audit of the merged phase, its
fixes and the final doc pass. The table and the notes below are the plan as it was run. Section 12 has what the groups
settled.

| Group | Classes | Files (owned) | Starts after |
|---|---|---|---|
| F1 Core | the streams, Lifetime, ObjectStreamTest | `Core/RTTI.zig`, `Core/Factory.zig`, `Core/Lifetime.zig`, `Core/StreamIn.zig` (switch to Lifetime), `ObjectStream/*.zig`, `ZoltTests/ObjectStream/*`, `ConfigurationString.zig`, PortingGuide rows | the prototype |
| F2 Migration | the 7 roots, all pending declarations, `createDefault` where missing | every class file of section 1 (one generated change), `RegisterTypes.zig`, `Core/StreamUtils.zig` (new), `Shape.zig` (helpers moved), the test and parity user type modules, `tools/gen_serializable.py` (new) | the prototype |
| F3 Harness | P1 to P7, P9, the oracle step in CI | `ZoltParity/ObjectStream/*`, `tools/port_status.py` | the prototype |
| G1 Shapes | ShapeSettings, Convex, Compound (+ SubShapeSettings), Decorated, 17 leaf settings (22) | the 21 shape files | F1, F2, F3 |
| G2 Bodies and scene | BodyCreationSettings, MassProperties, CollisionGroup, GroupFilter, GroupFilterTable, PhysicsMaterial, PhysicsMaterialSimple, PhysicsScene (+ ConnectedConstraint) (9) | `BodyCreationSettings.zig`, `MassProperties.zig`, `CollisionGroup.zig`, `GroupFilter.zig`, `GroupFilterTable.zig`, `PhysicsMaterial.zig`, `PhysicsMaterialSimple.zig`, `PhysicsScene.zig` | F1, F2, F3 |
| G3 Constraints | ConstraintSettings, TwoBody, the 12 settings, PathConstraintPath, Hermite (+ Point) (17) | `Physics/Constraints/*.zig` | F1, F2, F3 |
| G4 Vehicles | 13 vehicle classes, LinearCurve (+ Point) (15) | `Physics/Vehicle/*.zig`, `Core/LinearCurve.zig` | F1, F2, F3 |
| G5 Skeleton and ragdoll | Skeleton (+ Joint), SkeletalAnimation (+ JointState, Keyframe, AnimatedJoint), RagdollSettings (+ Part, AdditionalConstraint) (9) | `Skeleton/Skeleton.zig`, `Skeleton/SkeletalAnimation.zig`, `Physics/Ragdoll/Ragdoll.zig` | F1, F2, F3 |
| G6 M9 | RagdollScene, `perf_ragdoll` | `ZoltPerformanceTest/*`, `ZoltParity/Physics/PerfRagdoll*`, `build.zig`, `.github/workflows/zolt.yml` | G1, G2, G3, G5 (the scene port can start earlier) |
| G7 Integration | the final checks and docs | `Docs/Zolt/*`, `Progress.md` | all |

- Done in the prototype: the 6 TypeDeclarations classes, SerializableObject, SpringSettings and MotorSettings.
- **F1** finishes the prototype:
  - the file variants of S10 and the default factory option;
  - StreamIn on Lifetime.zig;
  - the Zolt-only sweeps Z1 to Z4 (section 5);
  - the PortingGuide: the section 7 row of `JPH_DECLARE_SERIALIZABLE_*` and the rename table.
- **F2** is one mechanical change:
  - the `rtti` vtable entries, without `rtti_name`, `rttiHash` and `WheelSettings.RTTI`;
  - pending declarations for the 72 other classes, generated from Jolt's sources by `tools/gen_serializable.py`;
  - `factory_types`, `factory`, StreamUtils and the restores through the factory (S9);
  - the existing binary state parity must pass unchanged (P9).
  - The generator also has `--check`: it compares every Zolt declaration with Jolt's block (names, order, aliases, enum
    flags, kinds, bases). CI runs it through `port_status.py --check`.
- **F3** builds the harness of section 5 with no per-class code. It starts from the prototype's
  `ObjectStreamParity.zig`.
- **G6** works in this order:
  1. P7 on the assets in both precisions;
  2. `perf_ragdoll` stage 1 (the loaded objects) and stage 2 (the first updates);
  3. the hashes with `-t=max`, `-t=1` and `-t=2`, in both precisions;
  4. the Ragdoll steps in `zolt.yml` and the short run row of the inline hash test (values from the oracle).
- **G6 interface changes.** `PerformanceTestScene.stopTest` returns `Error!void`, because
  `Ragdoll.removeFromPhysicsSystem` allocates. A null ragdoll from `createRagdoll` is a panic ("out of bodies"), where
  Jolt dereferences null. `-s=Ragdoll` is `(4, 10, 0.6)`, `-s=RagdollSinglePile` is `(1, 160, 0.4)`, and Ragdoll is
  the default scene, as in Jolt.

**Order.**
1. F1, F2 and F3 in parallel. They share no file. F2 and F3 use the prototype's API, which is frozen.
2. G1, G2 and G3 in parallel (the M9 path).
3. G5 and G4 in parallel. G6 starts the scene port and the C++ side of `perf_ragdoll`.
4. G6 finishes after G5. Then G7.

The machine has 4 cores. Run at most one heavy build per agent and at most three groups at a time.

**Shared files.**
- `zolt.zig`, `unit_tests.zig`, `parity.zig`, `reference_sources.zig`: merged with the union driver, then
  `python3 tools/tidy_registry.py`.
- `RegisterTypes.zig` and the generic harness belong to F2 and F3. Groups do not edit them. A group that needs a change
  reports it.

## 5. Parity plan

The C++ side knows classes only through Jolt's RTTI, so the generic harness has no per-class code. Every layer runs in
single and double precision.

| # | Test | What it compares |
|---|---|---|
| P1 | Schema | One line per class of the C++ Factory after RegisterTypes against `RegisterTypes.factory`: name, hash, abstract flag, bases, and every attribute with its declared type text and primitive type. Also the IsKindOf matrix. The prototype's `jolt_os_class_schema` is the seed. |
| P2 | Defaults | Every creatable class: C++ `CreateObject` and Write against Zolt `create` and write, text and binary. This proves every default value that the old assets rely on. |
| P3 | Random streams | Streams generated from the C++ schema: arrays of 0 to 3 elements, shared and null references to random concrete subclasses, finite floats, strings with escapes, valid enum values. Both sides read and write again in both formats; each reads the other's output. |
| P4 | Old schemas | Mutations of P3 streams: dropped, reordered, renamed and retyped attributes, `vec3` / `dvec3`, a class declared twice (with other attributes), unknown classes and objects, abstract objects, null, dangling and wrong-type identifiers, a main object of the wrong class. |
| P5 | Truncation | Every prefix of streams up to 64 KB, and 2000 prefixes of terrain2.bof. The result and the bytes written again. |
| P6 | Number words | More than 10^5 words through C++ `IStringStream >> T` and `strtoul` against Zolt's parser. |
| P7 | Assets | Human.tof, dead_pose1.tof and terrain2.bof read by both sides, written again in both formats, and the SaveBinaryState bytes of the loaded objects. PhysicsScene by cross-read (S17). |
| P8 | Unit test object | ObjectStreamTest's object in both formats and both directions (done in the prototype). |
| P9 | Binary state | The existing binary state parity passes unchanged after S9. |
| P10 | `perf_ragdoll` | In its own files (`PerformanceTest/Layers.h` clashes with `UnitTests/Layers.h`). Stage 1: the loaded RagdollSettings after the four calculations, the sampled pose, the terrain after FixInvalidScales. Stage 2: the state after each of the first updates, in the thread matrix. |

**Rules for the C++ side.**
- The C++ side of P2 to P5 runs with allocation hooks that zero every block and defer frees until the read returns (a
  quarantine). So members that C++ leaves indeterminate are 0, as in Zolt (D9), and Jolt's use-after-free paths stay
  harmless.
- Exception: the six outside classes (Color, AABox, Triangle, IndexedTriangle, IndexedTriangleNoMaterial, Plane) use the
  global `new`, which the hooks do not see. P2 skips their defaults. The defaults of Plane, IndexedTriangleNoMaterial
  and IndexedTriangle are compared inside PlaneShapeSettings and MeshShapeSettings. Zolt-only tests pin the defaults of
  AABox, Triangle and Color.
- Every C++ read starts with `errno = 0`. Jolt's `ReadIdentifier` tests errno without clearing it, so an ERANGE of an
  earlier read fails the next identifier (D7). The prototype's parity met this case: a subnormal float in one stream
  made the next stream fail on the C++ side.
- Generators never change counts or string lengths (Jolt allocates without bound), never write bool bytes other than 0
  and 1, never form Ref cycles and never declare one attribute twice. Cases with a divergence (S16) are pinned by
  Zolt-only tests.
- Trace order is not compared. Jolt destroys unreferenced objects in hash map order, Zolt in creation order.

**The harness at the end of Phase 8.**
- P1 fails for a missing or pending class and for a class that reaches a pending one (S18). P2 to P7 fail instead of
  skipping such a class.
- P3 and P4 go on after a mismatch and list the classes that failed at the end.
- P7 always compares the SaveBinaryState bytes of the loaded objects.
- P10 (`perf_ragdoll`) runs the scene code itself on both sides (section 12). Stage 2 runs 4 thread configurations: the
  first runs 40 steps, the others 20.
- The sweeps Z2 to Z4 require every asset to read.
- `ObjectStreamZoltTests.zig` also pins Jolt behavior that the generators never produce: class names with a NUL byte in
  text, a second `write` on one output stream, an object of an undeclared class in binary, the defaults of AABox,
  Triangle and Color, and the classes that use the RTTI of their parent (S3).

**Zolt-only sweeps.**
- Z1: every creatable class of the factory: default, text, read, binary, read, text. The bytes must not change.
- Z2: `checkAllAllocationFailures` over reading and writing (the assets, the unit test object, every Z1 default).
- Z3: every prefix of Human.tof and of its binary form: an error or a success, never a leak.
- Z4: byte and word mutations with an allocator that fails large requests: never a crash, a leak or a safety panic.

**Bite proofs.** Each group breaks one declaration on purpose (swap two attributes, drop an alias, change a default) and
shows that P1, P2 or P3 fails. The commit message says so.

## 6. Reference build change

Done in the prototype (section 10), in `build.zig`:
- `-DJPH_OBJECT_STREAM` in `cpp_flags`, with a comment;
- `run_parity_tests.setCwd(b.path("."))`;
- the `jolt-performance-test` step;
- the module `objectstream_test` (ZoltTests/ObjectStream/ObjectStreamTest.zig) for the parity tests.

The first parity build after the change recompiles the C++ library (about 4 minutes).

## 7. Checklist for porters

- File headers: `//! Port of:` and `//! Status:`. Keep Jolt's doc and implementation comments. Remove the
  `TODO(serialization)` lines of the classes you finish.
- The declaration is the first thing in the struct. Its doc comment names Jolt's DECLARE and IMPLEMENT macros.
- Names, order and aliases are Jolt's, verbatim. Never derive them.
- Kind from the IMPLEMENT macro. `createDefault` for creatable classes that own an allocator.
- No new `.attrAs` or `.custom` without a decision in this document.
- Enums: Jolt's width (S13).
- An integer that Jolt declares with JPH_ADD_ENUM_ATTRIBUTE needs `.enumAttr`. `.attr` compiles (S5), so check a user
  class by hand. `tools/gen_serializable.py --check` and P1 check Jolt's classes.
- Parity: P1 and P2 for every class, P3 and P4 through the generic harness. A bite proof.
- Done criteria of Phase 8 (since Phase 9, CLAUDE.md adds the debug renderer configuration):
  - `zig build test` in Debug, double precision, ReleaseFast and `-Duse_llvm=false`;
  - `zig build parity` in Debug and double precision, filtered to `ObjectStream` and the area's binary state tests during
    the work; the full parity at the end of F1, F2 and G7 (30 to 40 minutes);
  - `zig fmt --check`;
  - `python3 tools/port_status.py --write`, then `--check` (it also runs `gen_serializable.py --check --complete`).
- Delete the worktree's `.zig-cache` at the end.

## 8. Out of scope (Phase 9)

Phase 9 ported the SoftBody and Hair items of this list (S17).

- SoftBody: SoftBodyCreationSettings, SoftBodySharedSettings and its nested classes, their physics, and
  `PhysicsScene.mSoftBodies` (S17).
- Hair: HairSettings with its 9 nested classes, `RegisterHair`, and the ComputeSystem RTTI.
- `JPH_RENAME_CLASS` (unused in Jolt; `.renamed` exists).
- A build option that compiles ObjectStream out.

## 9. Rejected alternatives

- **A run time Factory with a global `Register`.** It needs a global or an object passed everywhere. The comptime factory
  gives the same lookups and finds collisions at compile time.
- **Names derived from Zig names or `@typeName`.** They fail for `r g b a`, the aliases, acronyms and nested classes.
- **Reflection over all fields.** Jolt serializes a chosen subset, in a chosen order, with aliases.
- **A nullable `rtti` vtable entry during the migration.** Pending declarations kept it non-null (S18).
- **Exact emulation of every Jolt bug on malformed input** (repeated attributes, Ref cycles). It means use-after-free and
  leaks; D11 and D12 reject such streams instead.
- **A strict number grammar.** It would reject words that Jolt reads ("12abc"). S12 models the reference platform.
- **Porting the SoftBody schemas in Phase 8.** Only writing a PhysicsScene needs them; M9 only reads. They come with the
  rest of SoftBody.
- **Comparing trace order.** It needs Jolt's hash map order for no result.

## 10. The foundation prototype and the final state

**The final state (end of Phase 8).**

| File | Status |
|---|---|
| `Zolt/Core/RTTI.zig`, `Factory.zig`, `Lifetime.zig` (Zolt addition), `StreamUtils.zig` | complete |
| the 14 `Zolt/ObjectStream/*.zig` files | complete |
| `Zolt/Core/LinearCurve.zig`, `Zolt/ConfigurationString.zig` | complete |
| the declarations of the 81 classes, in 64 Zig files | complete: none pending, none missing (Progress.md, section Serialization) |
| `Zolt/RegisterTypes.zig` | `factory_types` (48 types) and `factory` |
| `Zolt/Physics/PhysicsScene.zig` | partial: `mSoftBodies` is Phase 9 (S17) |
| `ZoltTests/ObjectStream/ObjectStreamTest.zig` | the port of ObjectStreamTest.cpp (both tests) |
| `ZoltTests/ObjectStream/ObjectStreamZoltTests.zig`, `ObjectStreamSweeps.zig` | the pins of S16 and the sweeps Z1 to Z4 |
| `ZoltParity/ObjectStream/*` | P1 to P8 |
| `ZoltParity/Physics/PerfRagdollParity.zig` + `PerfRagdollReference.cpp` | P10 (`perf_ragdoll`) |
| `ZoltPerformanceTest/RagdollScene.zig` | the port of PerformanceTest/RagdollScene.h (M9) |
| `tools/gen_serializable.py` | `--write` (pending declarations) and `--check [--complete]` |

Phase 9 changed three rows (S17, S18). The declarations cover 104 classes, `factory_types` has Jolt's 49 types, and
`PhysicsScene.zig` is complete.

M9 is reached. ZoltPerformanceTest gives Jolt's Ragdoll hashes with `-q=LinearCast` and `-t=max`, `-t=1` and `-t=2`:
`0x3be6ea76c07a2ac8` in single precision and `0x8dd6bc501859a228` in double precision. Zolt's CI checks the six runs,
after the oracle step (S1). All five determinism hashes of `determinism_check.yml` now match.

**The prototype (at the design commit).**

| File | Status then |
|---|---|
| `Zolt/Core/RTTI.zig` | partial: the `rtti` vtable entries of the 7 roots were F2's |
| `Zolt/Core/Factory.zig` | partial: `RegisterTypes.factory` was F2's |
| `Zolt/Core/Lifetime.zig` | complete (StreamIn switched in F1) |
| `Zolt/ObjectStream/ObjectStreamIn.zig` | partial: the file variant and the default factory were F1's |
| the 13 other `Zolt/ObjectStream/*.zig` files | complete |
| `SpringSettings.zig`, `MotorSettings.zig` | declarations complete |
| `ZoltTests/ObjectStream/ObjectStreamTest.zig` | the port of ObjectStreamTest.cpp (both tests) |
| `ZoltParity/ObjectStream/ObjectStreamParity.zig` + `ObjectStreamReference.cpp` | the seed of P1, P3 and P4; P8 was done |

What the prototype showed against the C++ library built with `JPH_OBJECT_STREAM`, in single and double precision:
- the schemas (names, hashes, abstract flags, bases, attributes with their declared types) and the IsKindOf matrix of
  31 classes equaled Jolt's;
- MotorSettings with 300 random value sets: Zolt wrote Jolt's text and binary bytes; each side read the other's bytes
  and wrote them again unchanged, in both formats;
- the object of ObjectStreamTest.cpp (all primitives, Array, StaticArray, C arrays, nested arrays, a raw pointer, a Ref
  and a RefConst to one shared object, two bases with the second at a non-zero offset): the same in both directions,
  with reference count 2 and correct casts on both sides;
- ten old or odd MotorSettings streams (Human.tof's old schema, retyped, unknown and missing attributes, partial number
  words, a garbage word, a float that does not parse, another version, a skipped object, a class declared twice) gave
  the same result and bytes on both sides.

With the define, a filtered sample of the existing parity still passed unchanged: the 42 tests whose names contain
"binary state", "sRestoreFromBinaryState" or "struct" (the binary states, the struct layouts), and the HelloWorld
comparison. The full parity run with the define was F1's.

## 11. Settled after the synthesis

The lead settled the open questions of the synthesis:
- **Exhaustive body enums.** MotionType, MotionQuality and OverrideMassProperties stay exhaustive with D3. They are
  never written with an unnamed value, and the binary state of BodyCreationSettings already treats them this way.
- **The old user hooks go.** F2 removes `material_types`, `group_filter_types`, `constraint_settings_types`,
  `path_constraint_path_types` and `vehicle_controller_settings_types` without a transition period. No module outside
  this repository uses them. The compile error that names `serializable_types` stays.
- **M9 without soft bodies.** M9 only reads PhysicsScene, so the missing `mSoftBodies` (S17) does not block it.
- **D11, D12 and D14 stay.** They only change streams that Jolt cannot read safely, or that no writer produces.
- **F2 is one generated change.** No other branch edits the class files during F2, so it is not split.
- **Jolt's stale errno (D7).** It is a Jolt bug. The harness works around it. It may be reported upstream; that is not
  part of this port.

## 12. Settled in the Phase 8 groups

Reviews and tests of the groups found bugs and doc errors. Most were fixed before their group merged, the others before
the phase ended (see "Bugs found on the way"). After the last merge, a completeness audit checked the whole phase
against the Jolt sources. It confirmed 37 findings, and each one was fixed or documented. These points were settled on
the way. They add to section 11 and bind later phases that change these classes (Phase 9 added the SoftBody and Hair
schemas). The file headers have the details.

| Jolt behavior | Decision |
|---|---|
| `RotatedTranslatedShapeSettings` and `CompoundShapeSettings::SubShapeSettings` leave `mPosition` and `mRotation` uninitialized; `PathConstraintPathHermite::Point` leaves its members uninitialized | Zero (D9), so a default object writes the bytes of Jolt's zero filled object. For the two shape settings that means a zero rotation, not identity; their constructors with arguments still set both members (G1, G3) |
| `MeshShapeSettings::EBuildQuality` is an `int` enum; a value without a name asserts, then calls through a null splitter | `BuildQuality` is a non-exhaustive `enum(i32)` (S13). A stream can hold any value: it is read, kept and written back. Creating the shape panics with a message (D16, G1) |
| `Constraint` stores the step overrides as `uint8(inN)` and asserts `inN < 256` only when asserts are on | The same: the assert is guarded by `Core.enable_asserts`, and the value is truncated like Jolt's release build. Since G3 an object stream can set these settings to any uint32. The first port's unguarded assert and `@intCast` were undefined in ReleaseFast for 256 or more (G3 review) |
| `BodyCreationSettings` streams `mShape` (the settings), not `mShapePtr` | The custom accessor `ShapeAttribute` over the union `shape` (S5): settings are written, a Shape is written as null, a link stores the settings (G2) |
| `sReadObject<T>` returns a main object of any class derived from T; the caller deletes it through a `T *` | When T has no vtable, the main object must be exactly a T (D17). `ObjectStreamIn.read` still returns the dynamic class (audit) |
| A class without an RTTI of its own uses its parent's, and C++ writes it through a `Parent &` | Zolt writes the parent's sub object (`RTTI.RTTIOwner`), which need not be at offset 0. A cast to such a class, a read of it and a link to it are compile errors (S3, audit) |
| `RTTI::DestructObject` of an abstract class deletes through the virtual destructor; `IsAbstract` is `mCreate == nullptr \|\| mDestruct == nullptr` | The same. Only SerializableObject, which has no vtable in Zolt, cannot be destructed: a panic (S7, audit) |
| Jolt looks some class names up as a `const char *`, so they end at the first NUL | The same: the factory, the class of an attribute and the class of a skipped instance. The other lookups use the whole word (S11, audit) |
| `ObjectStreamOut::Write` uses `try_emplace` and always advances the identifier counter | The same: a second write of an object keeps its identifier (S11, audit) |
| The binary string length is `min((uint32)size, 0x7fffffff)` | The same: truncated to 32 bits first, then capped (S11, audit) |
| PhysicsScene's restore does an unchecked StaticCast of the constraint settings to TwoBodyConstraintSettings | "Failed to create instance of type" (D8, audit) |
| glibc reads a float word whose exact value is a subnormal | Zolt fails it like every subnormal (D14, widened by the audit to decimal words) |

**Bugs found on the way.** Each was fixed with a test that failed before:
- F3's P6 found that the number parser accepted words just below the smallest normal number, which glibc rejects
  (tininess after rounding, S12). `ObjectStreamTextIn.parseFloat` follows glibc since the F3 merge.
- The F1 review found that a skipped class that contains itself recursed until the stack overflowed (D15).
- The F3 review found that the C++ side of P2 to P7 ran without its allocation hooks: its factory setup installed Jolt's
  allocator again. Every hooked call now checks that its hooks are still installed.
- G2's allocation failure tests found a double free in the binary `writeString`. When the writer failed after the
  string entered the string table, its `errdefer` freed the key, and `deinit` freed it again. The fix came after the
  G2 merge.
- G5 and G6 found that `RagdollScene.destroy` asserted after a StopTest that ran out of memory (see below).
- The audit found the rows of the table marked "audit", missing compile checks (S5), a harness and a CI check that let
  a pending class pass (S18), CI path filters without `PerformanceTest/**` and `Assets/**`, and stale docs.

**RagdollScene (G6).** In addition to the interface changes of section 4:
- `new RagdollScene(...)` is `RagdollScene.create(allocator, io, ...)`. `io` reads the three assets through the file
  variants of ObjectStreamIn. A file that cannot be read prints Jolt's message, and Load returns false. Only running
  out of memory is an error.
- StartTest reserves the slot of a ragdoll before it creates the ragdoll, so a ragdoll in the system always has its
  reference. A failed StartTest removes the ragdolls that it added.
- A StopTest that cannot remove a ragdoll releases the ragdolls before it and keeps the others in the list, so that a
  second call removes them.
- `destroy` leaks the ragdolls that are still in the list. Their release would destroy their bodies, and main destroys
  the physics system before the scene. This only happens when StopTest ran out of memory and was not called again.

**Harness rules.**
- **P1 while classes were pending.** P1 compared a pending class by its header only. A class that only the attributes
  of pending classes reach was reported as such, not as missing, and P2 to P7 skipped such classes. The audit made the
  rule strict (S18).
- **The NUL rule of the cross-read.** Until Phase 9 a PhysicsScene was compared by cross-reading (S17). Jolt's binary
  reader cuts a string at its first NUL, also in Jolt's own bytes (S15). So when Jolt's text has a NUL byte and the
  direct binary comparison fails, the binary bytes must equal what Jolt writes after reading its own binary bytes.
  Text, and binary without a NUL, keep the direct comparison. P4 found the case after G1 to G3 merged: a
  PhysicsMaterialSimple debug name with a NUL.
- **perf_ragdoll (P10).** Both sides run the scene code itself:
  - The C++ side includes `PerformanceTest/Layers.h`, `PerformanceTestScene.h` and `RagdollScene.h` in the namespace
    `PerfRagdoll`. PerformanceTest's Layers.h defines the same class names as UnitTests/Layers.h, which other references
    of the parity library include.
  - The test reads private members of the scene, so `private` is defined as `public` (`#define private public`) while
    RagdollScene.h is included. All Jolt headers are included before that.
  - The Zig side imports `ZoltPerformanceTest/RagdollScene.zig` as the module `perf_ragdoll_scene` (build.zig).
- **The working directory.** The parity run and the test run of ZoltPerformanceTest start in the repository root, so
  that they find `Assets/`. The unit test run has no working directory of its own, so `zig build test` must be started
  in the repository root for the sweeps Z2 to Z4.
- **The inline hash test.** ZoltPerformanceTest's inline hash test has a Ragdoll row of 5 updates. Its values come from
  the oracle (`zig build jolt-performance-test -- -q=LinearCast -t=1 -s=Ragdoll -i=5`).
- **The final check.** `tools/gen_serializable.py` skipped PhysicsScene's "mSoftBodies" until Phase 9 (S17).
  `port_status.py --check` fails on a pending or undeclared class (S18).
