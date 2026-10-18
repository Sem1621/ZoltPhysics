//! Port of: Jolt/ObjectStream/SerializableAttributeTyped.h
//! Status: complete
//!
//! AddSerializableAttributeTyped<MemberType>: the serialization functions of a JPH_ADD_ATTRIBUTE member, generated
//! once per member type from the codec that ObjectStream.zig picks (the OS* overloads of the member type). Zolt adds the
//! custom attribute (`.custom`), whose member is not a field.

const std = @import("std");
const RTTI = @import("../Core/RTTI.zig").RTTI;
const serializable_attribute = @import("SerializableAttribute.zig");
const OSDataType = serializable_attribute.OSDataType;
const MemberType = serializable_attribute.MemberType;
const ObjectStream = @import("ObjectStream.zig");
const ObjectStreamIn = @import("ObjectStreamIn.zig").ObjectStreamIn;
const ObjectStreamOut = @import("ObjectStreamOut.zig").ObjectStreamOut;

/// The MemberType of members of type M (one constant per member type and declared class)
pub fn memberType(comptime M: type, comptime declared: ?type, comptime where: []const u8) *const MemberType {
    return &struct {
        const C = ObjectStream.Codec(M, declared, where);
        const value: MemberType = .{
            .array_depth = C.array_depth,
            .data_type = C.data_type,
            .class_rtti = C.class_rtti,
            .primitive_rtti = C.primitive_rtti,
            .read = read,
            .write = write,
        };
        fn read(in: *ObjectStreamIn, member: *anyopaque, source: OSDataType) ObjectStreamIn.Error!void {
            return C.read(in, @ptrCast(@alignCast(member)), source);
        }
        fn write(out: *ObjectStreamOut, member: *const anyopaque) ObjectStreamOut.Error!void {
            return C.write(out, @ptrCast(@alignCast(member)));
        }
    }.value;
}

/// The MemberType of a custom attribute of class T: the codec of `Accessor.Member`. A reference is linked to the class
/// and stored with `Accessor.set` at link resolution (a null identifier calls nothing); other members are read into
/// `Accessor.prepare(class)`. Writing writes `Accessor.get(class)`.
pub fn customMemberType(comptime T: type, comptime Accessor: type) *const MemberType {
    return &struct {
        const C = ObjectStream.Codec(Accessor.Member, null, @typeName(T) ++ " (custom attribute)");
        const Target = ObjectStream.RefTarget(Accessor.Member);
        const value: MemberType = .{
            .array_depth = C.array_depth,
            .data_type = C.data_type,
            .class_rtti = C.class_rtti,
            .primitive_rtti = C.primitive_rtti,
            .read = read,
            .write = write,
        };
        fn store(class: *anyopaque, object: ?*anyopaque, object_rtti: *const RTTI) void {
            const self: *T = @ptrCast(@alignCast(class));
            const target: ?*Target.? = if (object) |o| @ptrCast(@alignCast(object_rtti.castTo(o, RTTI.of(Target.?)).?)) else null;
            Accessor.set(self, target);
        }
        fn read(in: *ObjectStreamIn, class: *anyopaque, source: OSDataType) ObjectStreamIn.Error!void {
            if (Target) |R| {
                std.debug.assert(source == .instance);
                return in.readPointerData(RTTI.of(R), class, store, true);
            }
            const self: *T = @ptrCast(@alignCast(class));
            return C.read(in, Accessor.prepare(self), source);
        }
        fn write(out: *ObjectStreamOut, class: *const anyopaque) ObjectStreamOut.Error!void {
            const self: *const T = @ptrCast(@alignCast(class));
            const member: Accessor.Member = Accessor.get(self);
            return C.write(out, &member);
        }
    }.value;
}
