#include "pch.h"
#include "raytracing.h"

namespace sr::raytracing
{
    using namespace sr::math;
    using namespace sr::utilities;
    using namespace sr::structs;

    const char* get_ray_trace_job_name(uint32_t mode)
    {
        switch (mode)
        {
            case SR_TRACE_MODE_GGX: return SR_TRACE_MODE_STR_GGX;
            case SR_TRACE_MODE_RANDOM: return SR_TRACE_MODE_STR_RANDOM;
            case SR_TRACE_MODE_ALBEDO: return SR_TRACE_MODE_STR_ALBEDO;
            case SR_TRACE_MODE_NORMALS: return SR_TRACE_MODE_STR_NORMALS;
            case SR_TRACE_MODE_EMISSION: return SR_TRACE_MODE_STR_EMISSION;
        }

        return nullptr;
    }

    tracejobptr get_ray_trace_job(uint32_t mode)
    {
        switch (mode)
        {
            case SR_TRACE_MODE_GGX: return &ray_trace_job_ggx;
            case SR_TRACE_MODE_RANDOM: return &ray_trace_job_random;
            case SR_TRACE_MODE_ALBEDO: return &ray_trace_job_albedo;
            case SR_TRACE_MODE_NORMALS: return &ray_trace_job_normals;
            case SR_TRACE_MODE_EMISSION: return &ray_trace_job_emission;
        }

        return nullptr;
    }

    void ray_trace_warp(const raytracecontext& ctx, uint32_t xmin, uint32_t ymin, uint32_t xmax, uint32_t ymax)
    {
        surface surf;
        auto bounces = ctx.bounces > 0u ? ctx.bounces - 1u : 0u; 

        for (uint32_t x = xmin; x < xmax; ++x)
        for (uint32_t y = ymin; y < ymax; ++y)
        {
            auto raydir = get_origin_raydir(ctx, x, y);
            if (ray_trace(ctx.tree, ctx.origin, raydir, &surf))
            {
                store_pixel(ctx.pixels, x, y, ctx.width, ctx.job(ctx.tree, raydir, surf, ctx.samples, bounces));
            }
        }
    }


    float3 ray_trace_job_ggx(const kdtree* tree, const float3& view, const surface& surf, uint32_t samples, uint32_t bounces)
    {
        return surf.albedo * ray_gather_ggx_recursive(tree, view, surf, samples, bounces) + surf.emission;
    }

    float3 ray_trace_job_random(const kdtree* tree, const float3& view, const surface& surf, uint32_t samples, uint32_t bounces)
    {
        auto dither = ((uint32_t)rand() % 0xFFFFu) / (float)0xFFFFu;
        return surf.albedo * ray_gather_random_recursive(tree, view, surf, samples, bounces, dither) + surf.emission;
    }

    float3 ray_trace_job_normals(const kdtree* tree, const float3& view, const surface& surf, uint32_t samples, uint32_t bounces)
    {
        return surf.normal * 0.5f + float3(0.5f);
    }

    float3 ray_trace_job_albedo(const kdtree* tree, const float3& view, const surface& surf, uint32_t samples, uint32_t bounces)
    {
        return surf.albedo;
    }

    float3 ray_trace_job_emission(const kdtree* tree, const float3& view, const surface& surf, uint32_t samples, uint32_t bounces)
    {
        return surf.emission;
    }


    float3 ray_gather_ggx_recursive(const kdtree* tree, const float3& view, const surface& parent, uint32_t samples, uint32_t bounces)
    {
        auto N = parent.normal;
        auto V = float3(view) * -1.0f;
        auto O = parent.position + parent.normal * 1e-2f;

        float weight = 1e-4f;
        auto color = SR_FLOAT3_ZERO;
        surface surf;

        for (auto i = 0u; i < samples; ++i)
        {
            auto Xi = math::hammersley(i, samples);
            auto H = math::importance_sample_ggx(Xi, N, parent.roughness);
            auto L = normalize(reflect(V, H));
            float ndotl = dot(N, L);

            if (ndotl < 1e-2f || !ray_trace(tree, O, L, &surf))
            {
                continue;
            }

            if (bounces > 0)
            {
               surf.emission = surf.emission + ray_gather_ggx_recursive(tree, L, surf, samples / 4, bounces - 1);
            }

            color = color + parent.albedo * surf.emission * ndotl;
            weight += ndotl;
        }

        return color / weight;
    }

    float3 ray_gather_random_recursive(const kdtree* tree, const float3& view, const surface& parent, uint32_t samples, uint32_t bounces, float dither)
    {
        auto N = parent.normal;
        auto V = view * -1.0f;
        auto O = parent.position + parent.normal * 1e-2f;

        float weight = 1e-4f;
        auto color = SR_FLOAT3_ZERO;
        surface surf;

        for (auto i = 0u; i < samples; ++i)
        {
            auto H = math::random_direction_half_sphere(N, i, float(samples), dither);
            auto L = normalize(reflect(V, H));
            float ndotl = dot(N, L);

            if (ndotl < 1e-2f || !ray_trace(tree, O, L, &surf))
            {
                continue;
            }

            if (bounces > 0)
            {
                surf.emission = surf.emission + ray_gather_random_recursive(tree, L, surf, samples / 4, bounces - 1, dither);
            }

            color = color + parent.albedo * surf.emission * ndotl;
            weight += ndotl;
        }

        return color / weight;
    }


    bool ray_trace(const kdtree* tree, const float3& origin, const float3& direction, surface* surf)
    {
        auto mesh = tree->get_mesh();
        raycasthit hit{};

        if (!tree->raycast(origin, direction, &hit))
        {
            return false;
        }

        sample_surface(mesh, origin, direction, &hit, surf);
        return true;
    }

    bool ray_trace(const mesh* mesh, const float3& origin, const float3& direction, surface* surf)
    {
        raycasthit hit{};
        float distance = 1.0e+30f;
        float uv[2];

        for (auto i = 0u; i < mesh->indexCount; i += 3)
        {
            distance = 1.0e+30f;

            if (raycast_triangle(origin,
                                 direction,
                                 mesh->vertexPositions + mesh->indices[i + 0] * 3,
                                 mesh->vertexPositions + mesh->indices[i + 1] * 3,
                                 mesh->vertexPositions + mesh->indices[i + 2] * 3,
                                 &distance,
                                 uv)
                                 && distance < hit.distance)
            {
                hit.uv[0] = uv[0];
                hit.uv[1] = uv[1];
                hit.index = i;
                hit.distance = distance;
            }
        }

        if (hit.index == -1)
        {
            return false;
        }

        sample_surface(mesh, origin, direction, &hit, surf);
        return true;
    }

    void sample_surface(const structs::mesh* mesh, const float3& origin, const float3& direction, const raycasthit* hit, surface* surf)
    {
        float uvw[3] = { (1.0f - hit->uv[0] - hit->uv[1]), hit->uv[0], hit->uv[1] };

        uint32_t vidx[3] =
        {
            mesh->indices[hit->index + 0] * 3,
            mesh->indices[hit->index + 1] * 3,
            mesh->indices[hit->index + 2] * 3
        };

        surf->position = origin + direction * hit->distance;
        surf->emission = surf->albedo = surf->normal = SR_FLOAT3_ZERO;

        for (auto i = 0u; i < 3; ++i)
        for (auto j = 0u; j < 3; ++j)
        {
            surf->normal[i] += mesh->vertexNormals[vidx[j] + i] * uvw[j];
            surf->albedo[i] += mesh->vertexAlbedo[vidx[j] + i] * uvw[j];
            surf->emission[i] += mesh->vertexEmission[vidx[j] + i] * uvw[j];
        }

        surf->normal = normalize(surf->normal);
        surf->roughness = 0.95f;
    }

    void store_pixel(uint8_t* pixels, uint32_t x, uint32_t y, uint32_t w, const float3& color)
    {
        auto gamma = linear_to_gamma(color);
        pixels[(y * w + x) * 4 + 0] = (uint8_t)sr::math::fmin(gamma.x * 255.0f, 255.0f);
        pixels[(y * w + x) * 4 + 1] = (uint8_t)sr::math::fmin(gamma.y * 255.0f, 255.0f);
        pixels[(y * w + x) * 4 + 2] = (uint8_t)sr::math::fmin(gamma.z * 255.0f, 255.0f);
        pixels[(y * w + x) * 4 + 3] = 255;
    }

    float3 get_origin_raydir(const raytracecontext& ctx, uint32_t x, uint32_t y)
    {
        auto u = (x + 0.5f) / ctx.width;
        auto v = (y + 0.5f) / ctx.height;
        auto vpos = mul(ctx.invproj, float4(u * 2.0f - 1.0f, v * 2.0f - 1.0f, 1.0f, 1.0f));
        auto dir = mul(ctx.invview, float4(vpos.x, vpos.y, vpos.z, 0.0f));
        return normalize(float3(dir.x, dir.y, dir.z));
    }
}