// Internal four-pair arithmetic. No SIMD types escape into the public API.
#ifndef REACTPHYSICS3D_SPHERE_PAIR_SIMD_H
#define REACTPHYSICS3D_SPHERE_PAIR_SIMD_H

#include <reactphysics3d/mathematics/Vector3.h>

#if !defined(RP3D_DISABLE_SIMD) && !defined(IS_RP3D_DOUBLE_PRECISION_ENABLED)
    #if defined(__wasm_simd128__)
        #include <wasm_simd128.h>
        #define RP3D_SPHERE_PAIR_SIMD_WASM
    #elif defined(__aarch64__) || defined(_M_ARM64)
        #if defined(_MSC_VER) && !defined(__clang__)
            #include <arm64_neon.h>
        #else
            #include <arm_neon.h>
        #endif
        #define RP3D_SPHERE_PAIR_SIMD_NEON
    #elif defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
        #include <emmintrin.h>
        #define RP3D_SPHERE_PAIR_SIMD_SSE
    #endif
#endif

#if defined(RP3D_SPHERE_PAIR_SIMD_WASM) || defined(RP3D_SPHERE_PAIR_SIMD_NEON) || defined(RP3D_SPHERE_PAIR_SIMD_SSE)
#define RP3D_SPHERE_PAIR_SIMD

namespace reactphysics3d {
namespace detail {

class SpherePairFloat4 {
    public:
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
        using Native = v128_t;
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
        using Native = float32x4_t;
#else
        using Native = __m128;
#endif
        Native value;
        explicit SpherePairFloat4(Native v) : value(v) {}

        static SpherePairFloat4 set(float a, float b, float c, float d) {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            return SpherePairFloat4(wasm_f32x4_make(a, b, c, d));
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            const float values[4] = {a, b, c, d};
            return SpherePairFloat4(vld1q_f32(values));
#else
            return SpherePairFloat4(_mm_setr_ps(a, b, c, d));
#endif
        }

        void store(float* output) const {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            wasm_v128_store(output, value);
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            vst1q_f32(output, value);
#else
            _mm_storeu_ps(output, value);
#endif
        }

        SpherePairFloat4 operator+(SpherePairFloat4 other) const {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            return SpherePairFloat4(wasm_f32x4_add(value, other.value));
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            return SpherePairFloat4(vaddq_f32(value, other.value));
#else
            return SpherePairFloat4(_mm_add_ps(value, other.value));
#endif
        }

        SpherePairFloat4 operator-(SpherePairFloat4 other) const {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            return SpherePairFloat4(wasm_f32x4_sub(value, other.value));
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            return SpherePairFloat4(vsubq_f32(value, other.value));
#else
            return SpherePairFloat4(_mm_sub_ps(value, other.value));
#endif
        }

        SpherePairFloat4 operator*(SpherePairFloat4 other) const {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            return SpherePairFloat4(wasm_f32x4_mul(value, other.value));
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            return SpherePairFloat4(vmulq_f32(value, other.value));
#else
            return SpherePairFloat4(_mm_mul_ps(value, other.value));
#endif
        }

        SpherePairFloat4 sqrt() const {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            return SpherePairFloat4(wasm_f32x4_sqrt(value));
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            return SpherePairFloat4(vsqrtq_f32(value));
#else
            return SpherePairFloat4(_mm_sqrt_ps(value));
#endif
        }

        SpherePairFloat4 operator/(SpherePairFloat4 other) const {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            return SpherePairFloat4(wasm_f32x4_div(value, other.value));
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            return SpherePairFloat4(vdivq_f32(value, other.value));
#else
            return SpherePairFloat4(_mm_div_ps(value, other.value));
#endif
        }

        // Select without evaluating invalid reciprocal inputs in unused lanes.
        static SpherePairFloat4 selectLess(SpherePairFloat4 a, SpherePairFloat4 b,
                                          SpherePairFloat4 yes, SpherePairFloat4 no) {
#if defined(RP3D_SPHERE_PAIR_SIMD_WASM)
            return SpherePairFloat4(wasm_v128_bitselect(yes.value, no.value, wasm_f32x4_lt(a.value, b.value)));
#elif defined(RP3D_SPHERE_PAIR_SIMD_NEON)
            return SpherePairFloat4(vbslq_f32(vcltq_f32(a.value, b.value), yes.value, no.value));
#else
            const __m128 mask = _mm_cmplt_ps(a.value, b.value);
            return SpherePairFloat4(_mm_or_ps(_mm_and_ps(mask, yes.value), _mm_andnot_ps(mask, no.value)));
#endif
        }
};

struct SpherePairVector4 {
    SpherePairFloat4 x, y, z;

    static SpherePairVector4 gather(const Vector3* v) {
        return {SpherePairFloat4::set(v[0].x, v[1].x, v[2].x, v[3].x),
                SpherePairFloat4::set(v[0].y, v[1].y, v[2].y, v[3].y),
                SpherePairFloat4::set(v[0].z, v[1].z, v[2].z, v[3].z)};
    }

    SpherePairVector4 operator*(SpherePairFloat4 s) const { return {x * s, y * s, z * s}; }

    SpherePairVector4 unit() const {
        const auto length = ((x * x + y * y) + z * z).sqrt();
        const auto one = SpherePairFloat4::set(1, 1, 1, 1);
        const auto epsilon = SpherePairFloat4::set(MACHINE_EPSILON, MACHINE_EPSILON, MACHINE_EPSILON, MACHINE_EPSILON);
        // Vector3::getUnit() returns the input unchanged below epsilon.
        const auto denominator = SpherePairFloat4::selectLess(length, epsilon, one, length);
        return *this * (one / denominator);
    }

    void store(Vector3* output) const {
        float xs[4], ys[4], zs[4];
        x.store(xs); y.store(ys); z.store(zs);
        for (unsigned int lane = 0; lane < 4; lane++) output[lane] = Vector3(xs[lane], ys[lane], zs[lane]);
    }
};

}
}
#endif
#endif
