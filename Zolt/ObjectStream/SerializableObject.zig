//! Port of: Jolt/ObjectStream/SerializableObject.h, Jolt/ObjectStream/SerializableObject.cpp
//! Status: complete
//!
//! - SerializableObject is the root of the classes that can be pointed to. It has no fields, so Zolt's pattern A
//!   roots do not embed it: it is an RTTI base at offset 0 (a flattened prefix) of ShapeSettings, PhysicsMaterial,
//!   GroupFilter, ConstraintSettings, PathConstraintPath and VehicleControllerSettings. Jolt declares it
//!   ABSTRACT_BASE and implements it ABSTRACT: it is not creatable.
//! - JPH_DECLARE_SERIALIZATION_FUNCTIONS / JPH_IMPLEMENT_SERIALIZATION_FUNCTIONS of a class C by value (`C &`) are
//!   `InstanceCodec(C)`; the pointer forms (`C *&`) are the `.pointer` codec of ObjectStream.zig.

const std = @import("std");
const RTTI = @import("../Core/RTTI.zig").RTTI;
const OSDataType = @import("SerializableAttribute.zig").OSDataType;
const ObjectStreamIn = @import("ObjectStreamIn.zig").ObjectStreamIn;
const ObjectStreamOut = @import("ObjectStreamOut.zig").ObjectStreamOut;

/// Base class for all serializable objects
pub const SerializableObject = struct {
    /// JPH_DECLARE_SERIALIZABLE_ABSTRACT_BASE / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(SerializableObject)
    pub const rtti = RTTI.declare(@This(), .{ .name = "SerializableObject", .kind = .abstract });
};

/// A class with an RTTI, by value: OSIsType(C *) is `array_depth == 0 && Instance && class name == "C"`,
/// OSReadData(C &) is ReadClassData("C", &c), OSWriteData(const C &) is WriteClassData(JPH_RTTI(C), &c) and
/// OSWriteDataType(C *) writes `Instance` and the name
pub fn InstanceCodec(comptime M: type) type {
    return struct {
        pub const array_depth: u32 = 0;
        pub const data_type: OSDataType = .instance;
        pub const class_rtti: ?*const RTTI = RTTI.of(M);
        pub const primitive_rtti: ?*const RTTI = RTTI.of(M);

        pub fn read(in: *ObjectStreamIn, member: *M, source: OSDataType) ObjectStreamIn.Error!void {
            std.debug.assert(source == data_type);
            return in.readClassData(RTTI.of(M), member);
        }

        pub fn write(out: *ObjectStreamOut, member: *const M) ObjectStreamOut.Error!void {
            return out.writeClassData(RTTI.of(M), member);
        }
    };
}
