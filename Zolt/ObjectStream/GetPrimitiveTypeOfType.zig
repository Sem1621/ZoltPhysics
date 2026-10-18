//! Port of: Jolt/ObjectStream/GetPrimitiveTypeOfType.h
//! Status: complete
//!
//! GetPrimitiveTypeOfType: the RTTI of the element of a member type, which ObjectStreamOut declares and Factory
//! registers (`SerializableAttribute.getMemberPrimitiveType`). The member codecs of ObjectStream.zig evaluate it at
//! compile time. Jolt returns this RTTI through a function pointer to break the dependency loop of classes that refer
//! to each other; Zig resolves pointers to compile time constants lazily, so Zolt stores the RTTI pointer itself.

const RTTI = @import("../Core/RTTI.zig").RTTI;
const ObjectStream = @import("ObjectStream.zig");

/// Helper function to get the underlying RTTI type of a type (so e.g. Array<sometype> will return sometype).
/// `declared` is the class of `.attrAs` (Jolt's member type), which replaces the class of a reference.
pub fn getPrimitiveTypeOfType(comptime T: type, comptime declared: ?type) *const RTTI {
    if (comptime ObjectStream.ArrayListElement(T)) |E| return getPrimitiveTypeOfType(E, declared);
    if (comptime ObjectStream.StaticArrayElement(T)) |E| return getPrimitiveTypeOfType(E, declared);
    if (comptime ObjectStream.RefTarget(T)) |C| return RTTI.of(declared orelse C);
    switch (@typeInfo(T)) {
        .array => |a| return getPrimitiveTypeOfType(a.child, declared),
        .optional => |o| if (@typeInfo(o.child) == .pointer) return RTTI.of(declared orelse @typeInfo(o.child).pointer.child),
        else => {},
    }
    return RTTI.of(declared orelse T);
}
