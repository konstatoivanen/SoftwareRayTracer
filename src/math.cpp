#include "pch.h"
#include "math.h"

namespace sr::math
{
    quaternion euler_to_quaternion(const float3& euler)
    {
        auto h = euler * 0.5f * SR_FLOAT_DEG2RAD;
        auto c = float3(cosf(h.x), cosf(h.y), cosf(h.z));
        auto s = float3(sinf(h.x), sinf(h.y), sinf(h.z));

        return quaternion
        (
            s.x * c.y * c.z - c.x * s.y * s.z,
            c.x * s.y * c.z + s.x * c.y * s.z,
            c.x * c.y * s.z - s.x * s.y * c.z,
            c.x * c.y * c.z + s.x * s.y * s.z
        );
    }

    float2 normalize(const float2& v)
    {
        return v / sqrtf(v.x * v.x + v.y * v.y);
    }

    float3 normalize(const float3& v)
    {
        return v / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    float4 normalize(const float4& v)
    {
        return v / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
    }

    float4x4 matrix_inverse(const float4x4& m)
    {
        auto c00 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
        auto c02 = m[1][2] * m[3][3] - m[3][2] * m[1][3];
        auto c03 = m[1][2] * m[2][3] - m[2][2] * m[1][3];

        auto c04 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
        auto c06 = m[1][1] * m[3][3] - m[3][1] * m[1][3];
        auto c07 = m[1][1] * m[2][3] - m[2][1] * m[1][3];

        auto c08 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
        auto c10 = m[1][1] * m[3][2] - m[3][1] * m[1][2];
        auto c11 = m[1][1] * m[2][2] - m[2][1] * m[1][2];

        auto c12 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
        auto c14 = m[1][0] * m[3][3] - m[3][0] * m[1][3];
        auto c15 = m[1][0] * m[2][3] - m[2][0] * m[1][3];

        auto c16 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
        auto c18 = m[1][0] * m[3][2] - m[3][0] * m[1][2];
        auto c19 = m[1][0] * m[2][2] - m[2][0] * m[1][2];

        auto c20 = m[2][0] * m[3][1] - m[3][0] * m[2][1];
        auto c22 = m[1][0] * m[3][1] - m[3][0] * m[1][1];
        auto c23 = m[1][0] * m[2][1] - m[2][0] * m[1][1];

        auto f0 = float4(c00, c00, c02, c03);
        auto f1 = float4(c04, c04, c06, c07);
        auto f2 = float4(c08, c08, c10, c11);
        auto f3 = float4(c12, c12, c14, c15);
        auto f4 = float4(c16, c16, c18, c19);
        auto f5 = float4(c20, c20, c22, c23);

        auto v0 = float4(m[1][0], m[0][0], m[0][0], m[0][0]);
        auto v1 = float4(m[1][1], m[0][1], m[0][1], m[0][1]);
        auto v2 = float4(m[1][2], m[0][2], m[0][2], m[0][2]);
        auto v3 = float4(m[1][3], m[0][3], m[0][3], m[0][3]);

        auto i0 = (v1 * f0) - (v2 * f1) + (v3 * f2);
        auto i1 = (v0 * f0) - (v2 * f3) + (v3 * f4);
        auto i2 = (v0 * f1) - (v1 * f3) + (v3 * f5);
        auto i3 = (v0 * f2) - (v1 * f4) + (v2 * f5);

        auto signA = float4( +1, -1, +1, -1);
        auto signB = float4( -1, +1, -1, +1);
        auto inverse = float4x4(i0 * signA, i1 * signB, i2 * signA, i3 * signB);

        auto row0 =  float4(inverse[0][0], inverse[1][0], inverse[2][0], inverse[3][0]);
        auto dot0 = m[0] * row0;
        auto dot1 = (dot0.x + dot0.y) + (dot0.z + dot0.w);
        
        return inverse * (1.0f / dot1);
    }

    float4x4 matrix_trs(const float3& position, const quaternion& rotation, const float3& scale)
    {
        float qxx(rotation.x * rotation.x);
        float qyy(rotation.y * rotation.y);
        float qzz(rotation.z * rotation.z);
        float qxz(rotation.x * rotation.z);
        float qxy(rotation.x * rotation.y);
        float qyz(rotation.y * rotation.z);
        float qwx(rotation.w * rotation.x);
        float qwy(rotation.w * rotation.y);
        float qwz(rotation.w * rotation.z);

        float4x4 m = SR_FLOAT4X4_IDENTITY;

        m[3].x = position.x;
        m[3].y = position.y;
        m[3].z = position.z;

        m[0][0] = scale[0] * (1.0f - 2.0f * (qyy + qzz));
        m[0][1] = scale[0] * (2.0f * (qxy + qwz));
        m[0][2] = scale[0] * (2.0f * (qxz - qwy));

        m[1][0] = scale[1] * (2.0f * (qxy - qwz));
        m[1][1] = scale[1] * (1.0f - 2.0f * (qxx + qzz));
        m[1][2] = scale[1] * (2.0f * (qyz + qwx));

        m[2][0] = scale[2] * (2.0f * (qxz + qwy));
        m[2][1] = scale[2] * (2.0f * (qyz - qwx));
        m[2][2] = scale[2] * (1.0f - 2.0f * (qxx + qyy));

        return m;
    }

    float4x4 matrix_tr(const float3& position, const quaternion& rotation)
    {
        float qxx(rotation.x * rotation.x);
        float qyy(rotation.y * rotation.y);
        float qzz(rotation.z * rotation.z);
        float qxz(rotation.x * rotation.z);
        float qxy(rotation.x * rotation.y);
        float qyz(rotation.y * rotation.z);
        float qwx(rotation.w * rotation.x);
        float qwy(rotation.w * rotation.y);
        float qwz(rotation.w * rotation.z);

        float4x4 m = SR_FLOAT4X4_IDENTITY;

        m[3].x = position.x;
        m[3].y = position.y;
        m[3].z = position.z;

        m[0][0] = 1.0f - 2.0f * (qyy + qzz);
        m[0][1] = 2.0f * (qxy + qwz);
        m[0][2] = 2.0f * (qxz - qwy);

        m[1][0] = 2.0f * (qxy - qwz);
        m[1][1] = 1.0f - 2.0f * (qxx + qzz);
        m[1][2] = 2.0f * (qyz + qwx);

        m[2][0] = 2.0f * (qxz + qwy);
        m[2][1] = 2.0f * (qyz - qwx);
        m[2][2] = 1.0f - 2.0f * (qxx + qyy);
        return m;
    }

    float4x4 matrix_trs(const float3& position, const float3& euler, const float3& scale)
    {
        return matrix_trs(position, euler_to_quaternion(euler), scale);
    }

    float4x4 matrix_tr(const float3& position, const float3& euler)
    {
        return matrix_tr(position, euler_to_quaternion(euler));
    }

    float4x4 matrix_perspective(float fov, float aspect, float zNear, float zFar)
    {
        const float tanHalfFovy = (float)tan(fov * SR_FLOAT_DEG2RAD / 2.0f);
        float4x4 proj = SR_FLOAT4X4_ZERO;
        proj[0][0] = 1.0f / (aspect * tanHalfFovy);
        proj[1][1] = 1.0f / (tanHalfFovy);
        proj[2][2] = (zFar + zNear) / (zFar - zNear);
        proj[2][3] = 1.0;
        proj[3][2] = -(2.0f * zFar * zNear) / (zFar - zNear);
        return proj;
    }

    float4x4 matrix_viewprojection(float fov, float aspect, float zNear, float zFar, const float3& position, const float3& euler)
    {
        float4x4 v = matrix_inverse(matrix_tr(position, euler));
        float4x4 p = matrix_perspective(fov, aspect, zNear, zFar);
        return mul(p, v);
    }

    float4x4 mul(const float4x4& m0, const float4x4& m1)
    {
        float4x4 m;
        m.c0 = m0[0] * m1[0][0] + m0[1] * m1[0][1] + m0[2] * m1[0][2] + m0[3] * m1[0][3];
        m.c1 = m0[0] * m1[1][0] + m0[1] * m1[1][1] + m0[2] * m1[1][2] + m0[3] * m1[1][3];
        m.c2 = m0[0] * m1[2][0] + m0[1] * m1[2][1] + m0[2] * m1[2][2] + m0[3] * m1[2][3];
        m.c3 = m0[0] * m1[3][0] + m0[1] * m1[3][1] + m0[2] * m1[3][2] + m0[3] * m1[3][3];
        return m;
    }

    float4 mul(const float4x4& m, const float4& v)
    {
        return float4
        (
            m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2] + m[3][0] * v[3],
            m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2] + m[3][1] * v[3],
            m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2] + m[3][2] * v[3],
            m[0][3] * v[0] + m[1][3] * v[1] + m[2][3] * v[2] + m[3][3] * v[3]
        );
    }

    float4 mul(const float4& v, const float4x4& m)
    {
        return float4
        (
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3],
            m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3]
        );
    }

    void cross(const float* v0, const float* v1, float* outv)
    {
        outv[0] = v0[1] * v1[2] - v1[1] * v0[2];
        outv[1] = v0[2] * v1[0] - v1[2] * v0[0];
        outv[2] = v0[0] * v1[1] - v1[0] * v0[1];
    }

    float radical_inverse_vdc(uint32_t bits)
    {
        bits = (bits << 16u) | (bits >> 16u);
        bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
        bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
        bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
        bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
        return float(bits) * 2.3283064365386963e-10f; // / 0x100000000
    }

    float2 hammersley(uint32_t i, uint32_t n)
    {
        return float2(float(i) / float(n), radical_inverse_vdc(i));
    }

    float3 importance_sample_ggx(const float2& Xi, const float3& N, float roughness)
    {
        float a = roughness * roughness;

        float phi = 2.0f * 3.14159265f * Xi.x;
        float cosTheta = (float)sqrt((1.0f - Xi.y) / (1.0f + (a * a - 1.0f) * Xi.y));
        float sinTheta = (float)sqrt(1.0f - cosTheta * cosTheta);

        // from spherical coordinates to cartesian coordinates
        float3 H;
        H.x = (float)cos(phi) * sinTheta;
        H.y = (float)sin(phi) * sinTheta;
        H.z = cosTheta;

        // from tangent-space vector to world-space sample vector
        float3 up = abs(N.z) < 0.999f ? float3(0.0f, 0.0f, 1.0f) : float3(1.0f, 0.0f, 0.0f);
        float3 tangent = normalize(cross(up, N));
        float3 bitangent = cross(N, tangent);

        float3 sampleVec = tangent * H.x + bitangent * H.y + N * H.z;
        return normalize(sampleVec);
    }

    float3 random_direction_half_sphere(const float3& worldNormal, uint32_t index, float sampleCount, float dither)
    {
        auto fi = float(index) + dither;
        auto fiN = fi / sampleCount;
        auto longitude = 5.08320368996f * fi;
        auto latitude = asin(fiN * 2.0f - 1.0f);

        float3 kernel;
        kernel.x = (float)cos(latitude) * (float)cos(longitude);
        kernel.z = (float)cos(latitude) * (float)sin(longitude);
        kernel.y = (float)sin(latitude);
        kernel = faceforward(kernel, kernel, worldNormal * -1.0f);
        return normalize(kernel);
    }

    float linear_to_gamma(float value)
    {
        if (value <= 0.0F)
            return 0.0F;
        else if (value <= 0.0031308F)
            return 12.92F * value;
        else if (value < 1.0F)
            return 1.055F * (float)pow(value, 0.4166667F) - 0.055F;
        else
            return (float)pow(value, 0.45454545F);
    }

    float3 linear_to_gamma(const float3& color)
    {
        return float3(linear_to_gamma(color.x), linear_to_gamma(color.y), linear_to_gamma(color.z));
    }

    bool intersect_bounds(const bounds& bounds, const float3& origin, const float3& direction, float* outNear, float* outFar)
    {
        auto tmin = (bounds.bmin - origin) / direction;
        auto tmax = (bounds.bmax - origin) / direction;
        auto t0 = vmin(tmin, tmax);
        auto t1 = vmax(tmin, tmax);
        *outNear = fmax(fmax(t0.x, t0.y), t0.z);
        *outFar = fmin(fmin(t1.x, t1.y), t1.z);
        return *outNear < *outFar;
    }

    // Source: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
    bool raycast_triangle(const float3& o,
                         const float3& dir, 
                         const float* v0, 
                         const float* v1, 
                         const float* v2, 
                         float* dist, 
                         float* barycoords)
    { 
        float v0v1[3]{ v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2] };
        float v0v2[3]{ v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2] };
        float pvec[3];

        cross(&dir.x, v0v2, pvec);
        auto det = dot(v0v1, pvec);

        if (det < 1.0e-4f)
        {
            return false;
        }

        auto invDet = 1.0f / det; 
        float tvec[3] = { o[0] - v0[0], o[1] - v0[1], o[2] - v0[2] };
        barycoords[0] = dot(tvec, pvec) * invDet;

        if (barycoords[0] < 0 || barycoords[0] > 1)
        {
            return false;
        }

        cross(tvec, v0v1, pvec);
        barycoords[1] = dot(&dir.x, pvec) * invDet;
        
        if (barycoords[1] < 0 || barycoords[0] + barycoords[1] > 1)
        {
            return false;
        }

        *dist = dot(v0v2, pvec) * invDet; 
        return true; 
    
    } 
}
