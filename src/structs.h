#pragma once
#include "math.h"
#include <cstdint>
#include <string>

namespace sr::structs
{
    constexpr const static uint32_t SR_TRACE_MODE_RANDOM = 0u;
    constexpr const static uint32_t SR_TRACE_MODE_GGX = 1u;

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
}