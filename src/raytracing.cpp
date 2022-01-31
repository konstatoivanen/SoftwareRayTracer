#include "pch.h"
#include "raytracing.h"

namespace sr::raytracing
{
    using namespace sr::math;
    using namespace sr::utilities;
    using namespace sr::structs;

    void ray_trace_job_ggx(raytracecontext* ctx, uint32_t x, uint32_t y, uint32_t w, uint32_t h)
    {
        uint32_t xmin = x;
        uint32_t ymin = y;
        uint32_t xmax = x + w;
        uint32_t ymax = y + h;
        surface surf;
        auto bounces = ctx->bounces > 0u ? ctx->bounces - 1u : 0u; 

        for (x = xmin; x < xmax; ++x)
        for (y = ymin; y < ymax; ++y)
        {
            auto raydir = get_origin_raydir(ctx, x, y);

            if (!ray_trace(ctx->tree, &ctx->origin.x, &raydir.x, &surf))
            {
                continue;
            }
            
            auto color = surf.albedo * ray_gather_ggx_recursive(ctx->tree, &raydir.x, &surf, ctx->samples, bounces) + surf.emission;
            store_pixel(ctx->pixels, x, y, ctx->width, color);
        }
    }

    void ray_trace_job_random(raytracecontext* ctx, uint32_t x, uint32_t y, uint32_t w, uint32_t h)
    {
        uint32_t xmin = x;
        uint32_t ymin = y;
        uint32_t xmax = x + w;
        uint32_t ymax = y + h;
        surface surf;
        auto bounces = ctx->bounces > 0u ? ctx->bounces - 1u : 0u;

        for (x = xmin; x < xmax; ++x)
        for (y = ymin; y < ymax; ++y)
        {
            auto raydir = get_origin_raydir(ctx, x, y);

            if (!ray_trace(ctx->tree, &ctx->origin.x, &raydir.x, &surf))
            {
                continue;
            }

            auto dither = ((uint32_t)rand() % 0xFFFFu) / (float)0xFFFFu;
            auto color = surf.albedo * ray_gather_random_recursive(ctx->tree, &raydir.x, &surf, ctx->samples, bounces, dither) + surf.emission;
            store_pixel(ctx->pixels, x, y, ctx->width, color);
        }
    }

    bool ray_trace(const kdtree* tree, const float* origin, const float* direction, surface* surf)
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

    bool ray_trace(const mesh* mesh, const float* origin, const float* direction, surface* surf)
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

    colorRGB ray_gather_ggx_recursive(const kdtree* tree, const float* view, const surface* parent, uint32_t samples, uint32_t bounces)
    {
        auto N = parent->normal;
        auto V = float3(view) * -1.0f;

        float weight = 1e-4f;
        auto color = SR_FLOAT3_ZERO;
        surface surf;

        for (auto i = 0u; i < samples; ++i)
        {
            auto Xi = math::hammersley(i, samples);
            auto H = math::importance_sample_ggx(Xi, N, parent->roughness);
            auto L = normalize((H * 2.0f * dot(V, H)) - V);
            float NdotL = fmax(dot(N, L), 0.0f);

            if (NdotL <= 0.0f)
            {
                continue;
            }

            auto O = parent->position + L * 0.05f;

            if (!ray_trace(tree, &O.x, &L.x, &surf))
            {
                continue;
            }

            if (bounces > 0)
            {
               surf.emission = surf.emission + ray_gather_ggx_recursive(tree, &L.x, &surf, samples / 4, bounces - 1);
            }

            color = color + parent->albedo * surf.emission * NdotL;
            weight += NdotL;
        }

        return color / weight;
    }

    colorRGB ray_gather_random_recursive(const kdtree* tree, const float* view, const surface* parent, uint32_t samples, uint32_t bounces, float dither)
    {
        auto N = parent->normal;
        auto V = float3(view) * -1.0f;

        float weight = 1e-4f;
        auto color = SR_FLOAT3_ZERO;
        surface surf;

        for (auto i = 0u; i < samples; ++i)
        {
            auto H = math::random_direction(N, i, float(samples), dither);
            auto L = normalize((H * 2.0f * dot(V, H)) - V);
            float NdotL = fmax(dot(N, L), 0.0f);

            if (NdotL <= 0.0f)
            {
                continue;
            }

            auto O = parent->position + L * 0.05f;

            if (!ray_trace(tree, &O.x, &L.x, &surf))
            {
                continue;
            }

            if (bounces > 0)
            {
                surf.emission = surf.emission + ray_gather_random_recursive(tree, &L.x, &surf, samples / 4, bounces - 1, dither);
            }

            color = color + parent->albedo * surf.emission * NdotL;
            weight += NdotL;
        }

        return color / weight;
    }

    void sample_surface(const structs::mesh* mesh, const float* origin, const float* direction, const raycasthit* hit, surface* surf)
    {
        auto u = (1.0f - hit->uv[0] - hit->uv[1]);
        auto v = hit->uv[0];
        auto w = hit->uv[1];

        uint32_t vind[3] =
        {
            mesh->indices[hit->index + 0] * 3,
            mesh->indices[hit->index + 1] * 3,
            mesh->indices[hit->index + 2] * 3
        };

        surf->position = float3(origin) + float3(direction) * hit->distance;

        surf->normal = float3(mesh->vertexNormals + vind[0]) * u +
            float3(mesh->vertexNormals + vind[1]) * v +
            float3(mesh->vertexNormals + vind[2]) * w;

        surf->albedo = float3(mesh->vertexAlbedo + vind[0]) * u +
            float3(mesh->vertexAlbedo + vind[1]) * v +
            float3(mesh->vertexAlbedo + vind[2]) * w;

        surf->emission = float3(mesh->vertexEmission + vind[0]) * u +
            float3(mesh->vertexEmission + vind[1]) * v +
            float3(mesh->vertexEmission + vind[2]) * w;

        surf->normal = normalize(surf->normal);
        surf->roughness = 0.95f;
    }

    void store_pixel(unsigned char* pixels, uint32_t x, uint32_t y, uint32_t w, const colorRGB& color)
    {
        pixels[(y * w + x) * 4 + 0] = (unsigned char)sr::math::fmin(color.x * 255.0f, 255.0f);
        pixels[(y * w + x) * 4 + 1] = (unsigned char)sr::math::fmin(color.y * 255.0f, 255.0f);
        pixels[(y * w + x) * 4 + 2] = (unsigned char)sr::math::fmin(color.z * 255.0f, 255.0f);
        pixels[(y * w + x) * 4 + 3] = 255;
    }

    float4 get_origin_raydir(const raytracecontext* ctx, uint32_t x, uint32_t y)
    {
        auto u = (x + 0.5f) / ctx->width;
        auto v = (y + 0.5f) / ctx->height;
        auto vpos = mul(ctx->invproj, float4(u * 2.0f - 1.0f, v * 2.0f - 1.0f, 1.0f, 1.0f));
        return normalize(mul(ctx->invview, float4(vpos.x, vpos.y, vpos.z, 0.0f)));
    }
}