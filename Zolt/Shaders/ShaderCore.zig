//! Port of: Jolt/Shaders/ShaderCore.h
//! Status: complete
//!
//! The types that the shaders and the C++ code share (Docs/Zolt/RemainingFeaturesArchitecture.md R11, R12).
//! - ShaderCore.h maps the `JPH_` types to Jolt's math storage types for C++ (`JPH_float3` is `Float3`, `JPH_uint3` is
//!   `uint32[3]`, `JPH_Mat44` is `Float4[4]`), to HLSLToCPP's types for the CPU wrappers and to HLSL's types for the GPU.
//!   Their layouts are the same, so Zolt has one set: the extern structs of Compute/CPU/HLSLToCPP.zig. A struct that
//!   C++ code fills and a shader reads (HairStructs.zig, TestCompute.zig) is one extern struct.
//! - The macros have no Zig counterpart (Compute/CPU/ShaderWrapper.zig has the full list):
//!   - JPH_SHADER_CONSTANT(type, name, value) is a `pub const` of the shader file (`cFoo` is `foo`);
//!   - JPH_SHADER_CONSTANTS_BEGIN / _MEMBER / _END is an extern struct and the binding `hlsl.Constants(T)`; its members
//!     drop the `c` (`cNumStrands` is `num_strands`);
//!   - JPH_SHADER_STRUCT_BEGIN / _MEMBER / _END is an extern struct; its members drop the `m` (`mPosition` is
//!     `position`);
//!   - JPH_SHADER_BIND_BUFFER / JPH_SHADER_BIND_RW_BUFFER are the bindings `hlsl.Buffer(T)` / `hlsl.RWBuffer(T)`;
//!   - JPH_SHADER_FUNCTION_BEGIN(void, main, ...) is `pub fn main(b: *Bindings, tid: hlsl.uint3) void`;
//!   - JPH_IN(type) is a value parameter, JPH_OUT / JPH_IN_OUT are return values or pointers (porting guide section 4);
//!   - JPH_AtomicAdd is `hlsl.atomicAdd`.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");

pub const JPH_float = f32;
pub const JPH_float3 = hlsl.float3;
pub const JPH_float4 = hlsl.float4;
pub const JPH_uint = u32;
pub const JPH_uint3 = hlsl.uint3;
pub const JPH_uint4 = hlsl.uint4;
pub const JPH_int = i32;
pub const JPH_int3 = hlsl.int3;
pub const JPH_int4 = hlsl.int4;
/// xyz = imaginary part, w = real part
pub const JPH_Quat = hlsl.Quat;
/// xyz = normal, w = constant
pub const JPH_Plane = hlsl.Plane;
/// Matrix, column major
pub const JPH_Mat44 = hlsl.Mat44;
