//! Port of: Jolt/Math/Trigonometry.h
//! Status: complete
//!
//! Note that this file exists because std::sin etc. are not platform independent and will lead to
//! non-deterministic simulation. The same holds for std.math in Zig: always use these functions.

const math = @import("Math.zig");
const Vec4 = @import("Vec4.zig").Vec4;

/// Sine of x (input in radians)
pub fn sin(x: f32) f32 {
    return Vec4.replicate(x).sinCos().sin.getX();
}

/// Cosine of x (input in radians)
pub fn cos(x: f32) f32 {
    return Vec4.replicate(x).sinCos().cos.getX();
}

/// Tangent of x (input in radians)
pub fn tan(x: f32) f32 {
    return Vec4.replicate(x).tan().getX();
}

/// Arc sine of x (returns value in the range [-PI / 2, PI / 2])
/// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::asin
pub fn asin(x: f32) f32 {
    return Vec4.replicate(x).asin().getX();
}

/// Arc cosine of x (returns value in the range [0, PI])
/// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::acos
pub fn acos(x: f32) f32 {
    return Vec4.replicate(x).acos().getX();
}

/// An approximation of ACos, max error is 4.2e-3 over the entire range [-1, 1], is approximately 2.5x faster than ACos
pub fn acosApproximate(x: f32) f32 {
    // See: https://www.johndcook.com/blog/2022/09/06/inverse-cosine-near-1/
    // See also: https://seblagarde.wordpress.com/2014/12/01/inverse-trigonometric-functions-gpu-optimization-for-amd-gcn-architecture/
    // Taylor of cos(x) = 1 - x^2 / 2 + ...
    // Substitute x = sqrt(2 y) we get: cos(sqrt(2 y)) = 1 - y
    // Substitute z = 1 - y we get: cos(sqrt(2 (1 - z))) = z <=> acos(z) = sqrt(2 (1 - z))
    // To avoid the discontinuity at 1, instead of using the Taylor expansion of acos(x) we use acos(x) / sqrt(2 (1 - x)) = 1 + (1 - x) / 12 + ...
    // Since the approximation was made at 1, it has quite a large error at 0 meaning that if we want to extend to the
    // range [-1, 1] by mirroring the range [0, 1], the value at 0+ is not the same as 0-.
    // So we observe that the form of the Taylor expansion is f(x) = sqrt(1 - x) * (a + b x) and we fit the function so that f(0) = pi / 2
    // this gives us a = pi / 2. f(1) = 0 regardless of b. We search for a constant b that minimizes the error in the range [0, 1].
    const abs_x = math.min(@abs(x), 1.0); // Ensure that we don't get a value larger than 1
    const val = @sqrt(1.0 - abs_x) * (math.pi / 2.0 - 0.175394 * abs_x);

    // Our approximation is valid in the range [0, 1], extend it to the range [-1, 1]
    return if (x < 0) math.pi - val else val;
}

/// Arc tangent of x (returns value in the range [-PI / 2, PI / 2])
pub fn atan(x: f32) f32 {
    return Vec4.replicate(x).atan().getX();
}

/// Arc tangent of y / x using the signs of the arguments to determine the correct quadrant (returns value in the range [-PI, PI])
pub fn atan2(y: f32, x: f32) f32 {
    return Vec4.atan2(Vec4.replicate(y), Vec4.replicate(x)).getX();
}
