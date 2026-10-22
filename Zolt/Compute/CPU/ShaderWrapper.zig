//! Port of: Jolt/Compute/CPU/ShaderWrapper.h, Jolt/Compute/CPU/WrapShaderBegin.h, Jolt/Compute/CPU/WrapShaderBindings.h, Jolt/Compute/CPU/WrapShaderEnd.h
//! Status: complete
//!
//! The wrapper that lets the CPU compute backend call a shader (Docs/Zolt/RemainingFeaturesArchitecture.md R12).
//! - `ShaderWrapper` is a pattern A root with Jolt's two virtual functions `bind` and `main` and the generated
//!   destructor. Jolt deletes the wrapper through a plain pointer (`delete mWrapper`), so the root has no reference
//!   count: the queue calls `destroy`.
//! - Jolt generates one wrapper class per shader with three macro headers around the shader code (WrapShaderBegin.h,
//!   the .hlsl file, WrapShaderBindings.h, the Bindings.h file, WrapShaderEnd.h). `ShaderWrapper.Of(Shader)` generates
//!   it from a Zig shader file (Zolt/Shaders):
//!   - the shader declares `pub const name` (JPH_SHADER_NAME), `pub const Bindings` and
//!     `pub fn main(b: *Bindings, tid: hlsl.uint3) void`;
//!   - `Bindings` has one field per binding with the HLSL binding name and a type of HLSLToCPP.zig:
//!     `hlsl.Constants(T)` (JPH_SHADER_CONSTANTS_BEGIN, the block is copied at bind time like Jolt's `memcpy`),
//!     `hlsl.Buffer(T)` (JPH_SHADER_BIND_BUFFER) or `hlsl.RWBuffer(T)` (JPH_SHADER_BIND_RW_BUFFER);
//!   - `bind` is an `inline for` over the fields (Jolt's `switch (HashString(inName))`), `main` calls `Shader.main`;
//!   - `create` is `sCreate`, the function that the registry of ComputeSystemCPU stores (ShaderWrapper.Create).
//! - JPH_SHADER_WRAPPER_FUNCTION / JPH_REGISTER_SHADER(sys, X) is `ComputeSystemCPU.registerShader(X)`.
//! - The other macros of WrapShaderBegin.h have no Zig counterpart: JPH_SHADER_CONSTANT is a `pub const` of the shader
//!   file, JPH_SHADER_STRUCT_* are extern structs. JPH_IN is a value parameter, JPH_OUT is a return value and
//!   JPH_IN_OUT is a pointer parameter (porting guide section 4).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const hlsl = @import("HLSLToCPP.zig");

/// Wraps a compute shader to allow calling it from Zig
pub const ShaderWrapper = struct {
    /// `ShaderWrapper *(*)()`: the factory function of a wrapper (`sCreate`). Zolt: it allocates with the allocator it
    /// receives, and the wrapper frees itself with it.
    pub const Create = *const fn (allocator: Allocator) Allocator.Error!*ShaderWrapper;

    pub const VTable = struct {
        /// Destructor (generated destructor chain)
        deinit: *const fn (self: *ShaderWrapper) void,
        /// delete this (generated)
        destroy: *const fn (self: *ShaderWrapper) void,
        /// Bind buffer to shader
        bind: *const fn (self: *ShaderWrapper, name: []const u8, data: []align(16) u8) void,
        /// Execute a single shader thread
        main: *const fn (self: *ShaderWrapper, thread_id: hlsl.uint3) void,
    };

    vtable: *const VTable,
    /// Frees this wrapper (Jolt: `new` / `delete`)
    allocator: Allocator,

    /// All virtual functions with a body are pure (an abstract class)
    pub const impl = struct {};

    /// Constructor, called by a wrapper class with its most derived type
    pub fn init(comptime T: type, allocator: Allocator) ShaderWrapper {
        return .{ .vtable = virtual.vtablePtr(VTable, T), .allocator = allocator };
    }

    /// Bind buffer to shader (`inData`, `inSize`: the bytes of the buffer)
    pub fn bind(self: *ShaderWrapper, name: []const u8, data: []align(16) u8) void {
        self.vtable.bind(self, name, data);
    }

    /// Execute a single shader thread
    pub fn main(self: *ShaderWrapper, thread_id: hlsl.uint3) void {
        self.vtable.main(self, thread_id);
    }

    /// delete this: runs the destructor and frees the wrapper
    pub fn destroy(self: *ShaderWrapper) void {
        self.vtable.destroy(self);
    }

    /// The wrapper class of a Zig shader (WrapShaderBegin.h, WrapShaderBindings.h and WrapShaderEnd.h around the
    /// shader code). `Shader` declares `name`, `Bindings` and `main` (see the file header).
    pub fn Of(comptime Shader: type) type {
        comptime checkShader(Shader);

        return struct {
            const Self = @This();

            pub const overrides = .{ .bind, .main };

            base: ShaderWrapper,
            /// The constant blocks and buffers of the shader, all zero / empty until bound (Jolt: `new X()`
            /// value-initializes the members, the buffer pointers start as nullptr)
            bindings: Shader.Bindings = .{},

            /// Factory function to create a shader wrapper for this shader (sCreate)
            pub fn create(allocator: Allocator) Allocator.Error!*ShaderWrapper {
                const self = try allocator.create(Self);
                self.* = .{ .base = .init(Self, allocator) };
                return &self.base;
            }

            /// Bind a buffer to the shader: Jolt's `switch (HashString(inName))` over the bindings
            pub fn bind(self: *Self, name: []const u8, data: []align(16) u8) void {
                inline for (@typeInfo(Shader.Bindings).@"struct".fields) |field| {
                    if (std.mem.eql(u8, name, field.name)) {
                        @field(self.bindings, field.name).bind(data);
                        return;
                    }
                }

                // JPH_ASSERT(false, "Buffer cannot be bound to this shader"), Jolt's release build ignores the bind
                if (Core.enable_asserts) @panic("Buffer cannot be bound to this shader");
            }

            /// Execute a single shader thread (the shader's main function)
            pub fn main(self: *Self, thread_id: hlsl.uint3) void {
                Shader.main(&self.bindings, thread_id);
            }
        };
    }
};

/// Readable compile errors for a shader file that does not follow the contract of R12
fn checkShader(comptime Shader: type) void {
    if (!@hasDecl(Shader, "name")) @compileError(@typeName(Shader) ++ " must declare `pub const name` (JPH_SHADER_NAME)");
    if (!@hasDecl(Shader, "Bindings")) @compileError(@typeName(Shader) ++ " must declare `pub const Bindings = struct { ... }`");
    if (!@hasDecl(Shader, "main") or @TypeOf(Shader.main) != fn (*Shader.Bindings, hlsl.uint3) void)
        @compileError(@typeName(Shader) ++ " must declare `pub fn main(b: *Bindings, tid: hlsl.uint3) void`");
    for (@typeInfo(Shader.Bindings).@"struct".fields) |field| {
        if (!@hasDecl(field.type, "bind"))
            @compileError(@typeName(Shader) ++ ".Bindings." ++ field.name ++ " must be a hlsl.Constants, hlsl.Buffer or hlsl.RWBuffer");
        if (field.defaultValue() == null)
            @compileError(@typeName(Shader) ++ ".Bindings." ++ field.name ++ " needs a default value (`= .{}`)");
    }
}

test "ShaderWrapper: bind by name, main, destroy" {
    const testing = std.testing;

    const Shader = struct {
        pub const name = "TestShader";

        const Context = extern struct { add: u32, num: u32 };

        pub const Bindings = struct {
            gContext: hlsl.Constants(Context) = .{},
            gInput: hlsl.Buffer(u32) = .{},
            gOutput: hlsl.RWBuffer(u32) = .{},
        };

        pub fn main(b: *Bindings, tid: hlsl.uint3) void {
            if (tid.x >= b.gContext.value.num)
                return;
            b.gOutput.set(tid.x, b.gInput.get(tid.x) +% b.gContext.value.add +% tid.y * 100);
        }
    };

    const wrapper = try ShaderWrapper.Of(Shader).create(testing.allocator);
    defer wrapper.destroy();

    var context: [2]u32 align(16) = .{ 5, 3 };
    var input: [4]u32 align(16) = .{ 1, 2, 3, 4 };
    var output: [4]u32 align(16) = @splat(0);
    wrapper.bind("gContext", std.mem.sliceAsBytes(&context));
    wrapper.bind("gInput", std.mem.sliceAsBytes(&input));
    wrapper.bind("gOutput", std.mem.sliceAsBytes(&output));

    // The constants were copied at bind time
    context[0] = 1000;

    for (0..4) |x|
        wrapper.main(.init(@intCast(x), 1, 0));
    try testing.expectEqualSlices(u32, &.{ 106, 107, 108, 0 }, &output);

    // An unknown name asserts, Jolt's release build ignores the bind
    if (!Core.enable_asserts) wrapper.bind("gUnknown", std.mem.sliceAsBytes(&input));
}
