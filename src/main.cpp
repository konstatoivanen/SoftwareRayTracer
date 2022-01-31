#include "pch.h"
#include "math.h"
#include "structs.h"
#include "raytracing.h"
#include "threadpool.h"
#include "fileio.h"
#include <filesystem>

#ifdef _DEBUG
    #define _CRTDBG_MAP_ALLOC  
    #include <stdlib.h>  
    #include <crtdbg.h>  
#endif 

using namespace sr::math;
using namespace sr::structs;
using namespace sr::utilities;
using namespace sr::raytracing;
using namespace sr::raytracing;

typedef std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double>> timestamp;

int main(int argc, char** argv)
{
#ifdef _DEBUG
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    config cfg;
    if (fileio::load_config("res/config.cfg", &cfg) != 0)
    {
        return -1;
    }

    mesh mesh;
    if (fileio::load_mesh(cfg.meshPath.c_str(), &mesh, 0.01f) != 0)
    {
        return -1;
    }

    printf("Loaded mesh: %s \n", cfg.meshPath.c_str());
    timestamp timeStart = std::chrono::steady_clock::now();

    kdtree tree(&mesh);

    printf("Built KD Tree. Depth: %i, Node count: %i \n", tree.get_depth(), tree.get_node_count());
    
    raytracecontext ctx;
    ctx.samples = cfg.samples;
    ctx.bounces = cfg.bounces;
    ctx.width = cfg.width;
    ctx.height = cfg.height;
    ctx.pixels = reinterpret_cast<unsigned char*>(calloc(4u * ctx.width * ctx.height, sizeof(char)));
    ctx.tree = &tree;
    ctx.origin = float3(0.0f, 2.5f, 5.0f);
    ctx.invproj = matrix_inverse(matrix_perspective(60.0f, ctx.width / (float)ctx.height, 0.1f, 100.0f));
    ctx.invview = matrix_inverse(matrix_tr(ctx.origin, float3(0.0f, 180.0f, 0.0f)));
    
    auto threadPool = new sr::utilities::threadpool();

    printf("Queueing %i jobs. \n", (ctx.width / cfg.groupSize) * (ctx.height / cfg.groupSize));

    for (uint32_t x = 0u; x < ctx.width; x += cfg.groupSize)
    for (uint32_t y = 0u; y < ctx.height; y += cfg.groupSize)
    {
        auto gw = (ctx.width - x) < cfg.groupSize ? (ctx.width - x) : cfg.groupSize;
        auto gh = (ctx.height - y) < cfg.groupSize ? (ctx.height - y) : cfg.groupSize;

        switch (cfg.mode)
        {
            case SR_TRACE_MODE_GGX:
                threadPool->queue_job(std::bind(ray_trace_job_random, &ctx, x, y, gw, gh));
                break;

            case SR_TRACE_MODE_RANDOM:
                threadPool->queue_job(std::bind(ray_trace_job_random, &ctx, x, y, gw, gh));
                break;
        }
    }
    
    delete threadPool;

    timestamp timeEnd = std::chrono::steady_clock::now();
    auto elapsed = (timeEnd - timeStart).count();

    printf("Tracing complete. Elapsed: %4.2f ms \n", elapsed * 1000.0f);

    auto filename = std::string("Screenshot0.bmp");
    auto index = 0;

    while (std::filesystem::exists(filename))
    {
        filename = std::string("Screenshot") + std::to_string(++index) + std::string(".bmp");
    }

    fileio::write_bmp(filename.c_str(), ctx.pixels, ctx.width, ctx.height);
    fileio::unload_mesh(&mesh);
    free(ctx.pixels);

    printf("Wrote file: %s \n", filename.c_str());
}
