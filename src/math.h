#pragma once

namespace sr::math
{
    struct float2
    {
        float x = 0.0f;
        float y = 0.0f;

        float2() = default;
        constexpr float2(float x, float y) : x(x), y(y) {}
        constexpr float2(float v) : x(v), y(v) {}

        inline float& operator[](size_t i) { return (&x)[i]; }
        inline constexpr const float& operator[](size_t i) const { return (&x)[i]; }
        inline constexpr float2 operator + (const float2& o) const { return { x + o.x, y + o.y }; }
        inline constexpr float2 operator - (const float2& o) const { return { x - o.x, y - o.y }; }
        inline constexpr float2 operator * (const float2& o) const { return { x * o.x, y * o.y }; }
        inline constexpr float2 operator / (const float2& o) const { return { x / o.x, y / o.y }; }
        inline constexpr float2 operator * (const float& o) const { return { x * o, y * o }; }
        inline constexpr float2 operator / (const float& o) const { return { x / o, y / o }; }
    };

    struct float3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        float3() = default;
        constexpr float3(float x, float y, float z) : x(x), y(y), z(z) {}
        constexpr float3(float v) : x(v), y(v), z(v) {}
        constexpr float3(const float* v) : x(v[0]), y(v[1]), z(v[2]) {}

        inline float& operator[](size_t i) { return (&x)[i]; }
        inline constexpr const float& operator[](size_t i) const { return (&x)[i]; }
        inline constexpr float3 operator + (const float3& o) const { return { x + o.x, y + o.y, z + o.z }; }
        inline constexpr float3 operator - (const float3& o) const { return { x - o.x, y - o.y, z - o.z }; }
        inline constexpr float3 operator * (const float3& o) const { return { x * o.x, y * o.y, z * o.z }; }
        inline constexpr float3 operator / (const float3& o) const { return { x / o.x, y / o.y, z / o.z }; }
        inline constexpr float3 operator * (const float& o) const { return { x * o, y * o, z * o }; }
        inline constexpr float3 operator / (const float& o) const { return { x / o, y / o, z / o }; }
    };

    struct float4
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 0.0f;

        float4() = default;
        constexpr float4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
        constexpr float4(float3 xyz, float w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
        constexpr float4(float v) : x(v), y(v), z(v), w(v) {}

        inline float& operator[](size_t i) { return (&x)[i]; }
        inline constexpr const float& operator[](size_t i) const { return (&x)[i]; }
        inline constexpr float4 operator + (const float4& o) const { return { x + o.x, y + o.y, z + o.z, w + o.w }; }
        inline constexpr float4 operator - (const float4& o) const { return { x - o.x, y - o.y, z - o.z, w - o.w }; }
        inline constexpr float4 operator * (const float4& o) const { return { x * o.x, y * o.y, z * o.z, w * o.w }; }
        inline constexpr float4 operator / (const float4& o) const { return { x / o.x, y / o.y, z / o.z, w / o.w }; }
        inline constexpr float4 operator * (const float& o) const { return { x * o, y * o, z * o, w * o }; }
        inline constexpr float4 operator / (const float& o) const { return { x / o, y / o, z / o, w / o }; }
    };

    struct float4x4
    {
        float4 c0;
        float4 c1;
        float4 c2;
        float4 c3;

        inline float4& operator[](size_t i) { return (&c0)[i]; }
        inline constexpr const float4& operator[](size_t i) const { return (&c0)[i]; }
        inline constexpr float4x4 operator * (const float& o) const { return { c0 * o, c1 * o, c2 * o, c3 * o }; }
    };

    struct bounds
    {
        float3 bmin{};
        float3 bmax{};
    };

    typedef float4 quaternion;
    typedef float4 color;

    constexpr float SR_FLOAT_PI = 3.14159274F;
    constexpr float SR_FLOAT_2PI = 2.0f * 3.14159274F;
    constexpr float SR_FLOAT_DEG2RAD = 0.0174532924F;
    constexpr float SR_FLOAT_RAD2DEG = 57.29578F;

    constexpr float2 SR_FLOAT2_ONE = { 1.0f, 1.0f };
    constexpr float3 SR_FLOAT3_ONE = { 1.0f, 1.0f, 1.0f };
    constexpr float4 SR_FLOAT4_ONE = { 1.0f, 1.0f, 1.0f, 1.0f };

    constexpr float2 SR_FLOAT2_ZERO = { 0.0f, 0.0f };
    constexpr float3 SR_FLOAT3_ZERO = { 0.0f, 0.0f, 0.0f };
    constexpr float4 SR_FLOAT4_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f };

    constexpr float2 SR_FLOAT2_UP = { 0.0f, 1.0f };
    constexpr float2 SR_FLOAT2_DOWN = { 0.0f, -1.0f };
    constexpr float2 SR_FLOAT2_LEFT = { -1.0f, 0.0f };
    constexpr float2 SR_FLOAT2_RIGHT = { 1.0f, 0.0f };

    constexpr float3 SR_FLOAT3_LEFT = { 1.0f,  0.0f,  0.0f };
    constexpr float3 SR_FLOAT3_RIGHT = { -1.0f,  0.0f,  0.0f };
    constexpr float3 SR_FLOAT3_UP = { 0.0f,  1.0f,  0.0f };
    constexpr float3 SR_FLOAT3_DOWN = { 0.0f, -1.0f,  0.0f };
    constexpr float3 SR_FLOAT3_FORWARD = { 0.0f,  0.0f,  1.0f };
    constexpr float3 SR_FLOAT3_BACKWARD = { 0.0f,  0.0f, -1.0f };

    constexpr color SR_COLOR_WHITE = { 1.0f, 1.0f, 1.0f, 1.0f };
    constexpr color SR_COLOR_GRAY = { 0.5f, 0.5f, 0.5f, 1.0f };
    constexpr color SR_COLOR_BLACK = { 0.0f, 0.0f, 0.0f, 1.0f };
    constexpr color SR_COLOR_CLEAR = { 0.0f, 0.0f, 0.0f, 0.0f };
    constexpr color SR_COLOR_RED = { 1.0f, 0.0f, 0.0f, 1.0f };
    constexpr color SR_COLOR_GREEN = { 0.0f, 1.0f, 0.0f, 1.0f };
    constexpr color SR_COLOR_BLUE = { 0.0f, 0.0f, 1.0f, 1.0f };
    constexpr color SR_COLOR_CYAN = { 0.0f, 1.0f, 1.0f, 1.0f };
    constexpr color SR_COLOR_MAGENTA = { 1.0f, 0.0f, 1.0f, 1.0f };
    constexpr color SR_COLOR_YELLOW = { 1.0f, 1.0f, 0.0f, 1.0f };

    constexpr float4x4 SR_FLOAT4X4_IDENTITY =
    {
        {1.0f,0.0f,0.0f,0.0f},
        {0.0f,1.0f,0.0f,0.0f},
        {0.0f,0.0f,1.0f,0.0f},
        {0.0f,0.0f,0.0f,1.0f}
    };

    constexpr float4x4 SR_FLOAT4X4_ZERO = { SR_FLOAT4_ZERO, SR_FLOAT4_ZERO, SR_FLOAT4_ZERO, SR_FLOAT4_ZERO };

    constexpr quaternion SR_QUATERNION_IDENTITY = { 1.0f, 0.0f, 0.0f, 0.0f };

    quaternion euler_to_quaternion(const float3& euler);

    float2 normalize(const float2& v);

    float3 normalize(const float3& v);

    float4 normalize(const float4& v);

    float4x4 matrix_inverse(const float4x4& matrix);

    float4x4 matrix_trs(const float3& position, const quaternion& rotation, const float3& scale);

    float4x4 matrix_tr(const float3& position, const quaternion& rotation);

    float4x4 matrix_trs(const float3& position, const float3& euler, const float3& scale);

    float4x4 matrix_tr(const float3& position, const float3& euler);

    float4x4 matrix_perspective(float fov, float aspect, float zNear, float zFar);

    float4x4 matrix_viewprojection(float fov, float aspect, float zNear, float zFar, const float3& position, const float3& euler);

    float4x4 mul(const float4x4& m0, const float4x4& m1);
    float4 mul(const float4x4& m, const float4& v);
    float4 mul(const float4& v, const float4x4& m);

    float3 cross(const float3& v0, const float3& v1);
    constexpr float dot(const float3& v0, const float3& v1) { return v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2]; }

    void cross(const float* v0, const float* v1, float* outv);
    constexpr float dot(const float* v0, const float* v1) { return v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2]; }
    constexpr float fmin(const float a, const float b) { return a < b ? a : b; }
    constexpr float fmax(const float a, const float b) { return a > b ? a : b; }
    constexpr float3 vmin(const float3& a, const float3& b) { return float3(fmin(a.x, b.x), fmin(a.y, b.y), fmin(a.z, b.z)); }
    constexpr float3 vmax(const float3& a, const float3& b) { return float3(fmax(a.x, b.x), fmax(a.y, b.y), fmax(a.z, b.z)); }

    float radical_inverse_vdc(uint32_t bits);
    float2 hammersley(uint32_t i, uint32_t n);
    float3 importance_sample_ggx(const float2& Xi, const float3& N, float roughness);
    float3 random_direction(const float3& worldNormal, uint32_t index, float sampleCount, float dither);
    float linear_to_gamma(float value);
    float3 linear_to_gamma(const float3& color);

    bool intersect_bounds(const bounds& bounds, const float* origin, const float* direction, float* outNear, float* outFar);

    bool raycast_triangle(const float* origin, 
                         const float* direction, 
                         const float* v0, 
                         const float* v1, 
                         const float* v2,
                         float* dist,
                         float* barycoords);
}