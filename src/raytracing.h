#pragma once
#include "structs.h"
#include "math.h"
#include "kdtree.h"

namespace sr::raytracing
{
    struct surface
    {
        math::float3 position = math::SR_FLOAT3_ZERO;
        math::float3 normal = math::SR_FLOAT3_ZERO;
        math::float3 albedo = math::SR_FLOAT3_ZERO;
        math::float3 emission = math::SR_FLOAT3_ZERO;
        float roughness = 0.0f;
    };

    struct raytracecontext
    {
        unsigned char* pixels = nullptr;
        uint32_t samples = 1u;
        uint32_t bounces = 0u;
        uint32_t width = 256u;
        uint32_t height = 256u;
        math::float3 origin = math::SR_FLOAT3_ZERO;
        math::float4x4 invproj = math::SR_FLOAT4X4_IDENTITY;
        math::float4x4 invview = math::SR_FLOAT4X4_IDENTITY;
        const utilities::kdtree* tree = nullptr;
    };

    void ray_trace_job_ggx(raytracecontext* ctx, uint32_t x, uint32_t y, uint32_t w, uint32_t h);

    void ray_trace_job_random(raytracecontext* ctx, uint32_t x, uint32_t y, uint32_t w, uint32_t h);

    bool ray_trace(const utilities::kdtree* tree, const float* origin, const float* direction, surface* surf);

    bool ray_trace(const structs::mesh* mesh, const float* origin, const float* direction, surface* surf);

    math::float3 ray_gather_ggx_recursive(const utilities::kdtree* tree,
        const float* view, 
        const surface* parent, 
        uint32_t samples,
        uint32_t bounces);

    math::float3 ray_gather_random_recursive(const utilities::kdtree* tree,
        const float* view,
        const surface* parent,
        uint32_t samples,
        uint32_t bounces,
        float dither);

    void sample_surface(const structs::mesh* mesh, const float* origin, const float* direction, const utilities::raycasthit* hit, surface* surf);

    void store_pixel(unsigned char* pixels, uint32_t x, uint32_t y, uint32_t w, const math::float3& color);

    math::float4 get_origin_raydir(const raytracecontext* ctx, uint32_t x, uint32_t y);
}