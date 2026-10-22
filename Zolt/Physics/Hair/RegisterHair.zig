//! Port of: Jolt/Physics/Hair/RegisterHair.h, Jolt/Physics/Hair/RegisterHair.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R14: Jolt's RegisterHair() registers HairSettings in the Factory at run
//! time ("call this function after RegisterTypes"). Zolt's factory is built at compile time (SerializationArchitecture.md
//! S8): an application adds `RegisterHair.serializable_types` to the `serializable_types` of its `zolt_user_types`
//! module, or builds a factory with `Factory.init(RegisterTypes.factory_types ++ ...)`. Like Jolt's, the factory then
//! also knows the classes of HairSettings' attributes. HairSettings::SkinPoint is not an attribute, so it is not
//! registered, like in Jolt.

const HairSettings = @import("HairSettings.zig").HairSettings;

/// The hair system is not registered by default: the classes that RegisterHair() registers
pub const serializable_types = .{HairSettings};

test "RegisterHair: HairSettings and its attribute classes, without SkinPoint" {
    const std = @import("std");
    const testing = std.testing;
    const Factory = @import("../../Core/Factory.zig").Factory;
    const RegisterTypes = @import("../../RegisterTypes.zig");

    const factory: Factory = comptime .init(RegisterTypes.factory_types ++ &[_]type{HairSettings});
    try testing.expect(RegisterTypes.factory.find("HairSettings") == null);
    try testing.expect(factory.find("HairSettings") == HairSettings.rtti);
    for ([_][]const u8{ "HairSettings::SVertex", "HairSettings::SStrand", "HairSettings::RStrand", "HairSettings::RVertex", "HairSettings::SVertexInfluence", "HairSettings::SkinWeight", "HairSettings::Material", "HairSettings::Gradient" }) |name|
        try testing.expect(factory.find(name) != null);
    try testing.expect(factory.find("HairSettings::SkinPoint") == null);
    try testing.expect(factory.find("HairSettings::SStrand").?.isKindOf(factory.find("HairSettings::RStrand").?));
}
