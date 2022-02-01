#pragma once
#include "math.h"
#include <cstdint>
#include <string>

namespace sr::structs
{
    constexpr const static uint32_t SR_TRACE_MODE_RANDOM = 0u;
    constexpr const static uint32_t SR_TRACE_MODE_GGX = 1u;
    constexpr const static uint32_t SR_TRACE_MODE_ALBEDO = 2u;
    constexpr const static uint32_t SR_TRACE_MODE_NORMALS = 3u;
    constexpr const static uint32_t SR_TRACE_MODE_EMISSION = 4u;

    struct config
    {
        std::string meshPath = "";
        uint32_t width = 256u;
        uint32_t height = 256u;
        uint32_t groupSize = 4u;
        uint32_t samples = 1024u;
        uint32_t bounces = 2u;
        uint32_t mode = SR_TRACE_MODE_RANDOM;
    };

    struct mesh
    {
        float* vertexPositions = nullptr;
        float* vertexNormals = nullptr;
        float* vertexAlbedo = nullptr;
        float* vertexEmission = nullptr;
        uint32_t* indices = nullptr;
        size_t indexCount = 0ull;
        size_t vertexCount = 0ull;
        math::bounds bounds{};
    };

    struct indexset
    {
        uint32_t position = 0u;
        uint32_t normal = 0u;
        uint32_t uv = 0u;

        inline bool operator < (const indexset& r) const noexcept
        {
            return memcmp(reinterpret_cast<const void*>(this), reinterpret_cast<const void*>(&r), sizeof(indexset)) < 0;
        }
    };

    struct raycasthit
    {
        int32_t index = -1;
        float distance = 1.0e+38f;
        float uv[2]{};
    };

    struct surface
    {
        math::float3 position = math::SR_FLOAT3_ZERO;
        math::float3 normal = math::SR_FLOAT3_ZERO;
        math::float3 albedo = math::SR_FLOAT3_ZERO;
        math::float3 emission = math::SR_FLOAT3_ZERO;
        float roughness = 0.0f;
    };

    struct facerange
    {
        uint32_t firstIndex = 0u;
        uint32_t indexCount = 0u;
    };
}