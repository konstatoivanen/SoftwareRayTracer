#pragma once
#include "structs.h"
#include "math.h"
#include "kdtree.h"

namespace sr::raytracing
{
    typedef math::float3 (*tracejobptr)(const utilities::kdtree*, const math::float3&, const structs::surface&, uint32_t, uint32_t);

    struct raytracecontext
    {
        uint8_t* pixels = nullptr;
        uint32_t samples = 1u;
        uint32_t bounces = 0u;
        uint32_t width = 256u;
        uint32_t height = 256u;
        math::float3 origin = math::SR_FLOAT3_ZERO;
        math::float4x4 invproj = math::SR_FLOAT4X4_IDENTITY;
        math::float4x4 invview = math::SR_FLOAT4X4_IDENTITY;
        const utilities::kdtree* tree = nullptr;
        tracejobptr job = nullptr;
    };

    const char* get_ray_trace_job_name(uint32_t mode);
    tracejobptr get_ray_trace_job(uint32_t mode);

    void ray_trace_warp(const raytracecontext& ctx, uint32_t xmin, uint32_t ymin, uint32_t xmax, uint32_t ymax);

    math::float3 ray_trace_job_ggx(const utilities::kdtree* tree, const math::float3& view, const structs::surface& surf, uint32_t samples, uint32_t bounces);
    math::float3 ray_trace_job_random(const utilities::kdtree* tree, const math::float3& view, const structs::surface& surf, uint32_t samples, uint32_t bounces);
    math::float3 ray_trace_job_normals(const utilities::kdtree* tree, const math::float3& view, const structs::surface& surf, uint32_t samples, uint32_t bounces);
    math::float3 ray_trace_job_albedo(const utilities::kdtree* tree, const math::float3& view, const structs::surface& surf, uint32_t samples, uint32_t bounces);
    math::float3 ray_trace_job_emission(const utilities::kdtree* tree, const math::float3& view, const structs::surface& surf, uint32_t samples, uint32_t bounces);

    math::float3 ray_gather_ggx_recursive(const utilities::kdtree* tree,
                                          const math::float3& view,
                                          const structs::surface& parent,
                                          uint32_t samples,
                                          uint32_t bounces);

    math::float3 ray_gather_random_recursive(const utilities::kdtree* tree,
                                             const math::float3& view,
                                             const structs::surface& parent,
                                             uint32_t samples,
                                             uint32_t bounces,
                                             float dither);

    bool ray_trace(const utilities::kdtree* tree, const math::float3& origin, const math::float3& direction, structs::surface* surf);
    bool ray_trace(const structs::mesh* mesh, const math::float3& origin, const math::float3& direction, structs::surface* surf);
    void sample_surface(const structs::mesh* mesh, const math::float3& origin, const math::float3& direction, const structs::raycasthit* hit, structs::surface* surf);
    void store_pixel(uint8_t* pixels, uint32_t x, uint32_t y, uint32_t w, const math::float3& color);
    math::float3 get_origin_raydir(const raytracecontext& ctx, uint32_t x, uint32_t y);
}