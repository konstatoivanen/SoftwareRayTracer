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
#if defined(_DEBUG) && defined(_WIN32)
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    config cfg;
    if (fileio::load_config("res/config.cfg", &cfg) != 0)
    {
        printf("Failed to load config at: res/config.cfg \n");
        return -1;
    }

    mesh mesh;
    if (fileio::load_mesh(cfg.meshPath.c_str(), &mesh, 0.01f) != 0)
    {
        printf("Failed to load mesh at: %s \n", cfg.meshPath.c_str());
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
    ctx.pixels = reinterpret_cast<uint8_t*>(calloc(4u * ctx.width * ctx.height, sizeof(uint8_t)));
    ctx.tree = &tree;
    ctx.origin = float3(0.0f, 2.5f, 5.0f);
    ctx.invproj = matrix_inverse(matrix_perspective(60.0f, ctx.width / (float)ctx.height, 0.1f, 100.0f));
    ctx.invview = matrix_inverse(matrix_tr(ctx.origin, float3(0.0f, 180.0f, 0.0f)));

    if ((ctx.job = get_ray_trace_job(cfg.mode)) == nullptr)
    {
        printf("Invalid job mode: %i", cfg.mode);
        return -1;
    }

    auto threadPool = threadpool();

    printf("Queueing %i jobs. \n", (ctx.width / cfg.groupSize) * (ctx.height / cfg.groupSize));

    for (uint32_t x = 0u; x < ctx.width; x += cfg.groupSize)
    for (uint32_t y = 0u; y < ctx.height; y += cfg.groupSize)
    {
        auto xmax = x + ((ctx.width - x) < cfg.groupSize ? (ctx.width - x) : cfg.groupSize);
        auto ymax = y + ((ctx.height - y) < cfg.groupSize ? (ctx.height - y) : cfg.groupSize);

        // @TODO Not a big fan of std::bind, refactor this to not require it.
        threadPool.queue_job(std::bind(ray_trace_warp, ctx, x, y, xmax, ymax));
    }
    
    threadPool.wait_all(10ul);

    timestamp timeEnd = std::chrono::steady_clock::now();

    printf("Tracing complete. Elapsed: %4.2f ms \n", (timeEnd - timeStart).count() * 1000.0f);

    fileio::unload_mesh(&mesh);

    auto filename = fileio::get_free_filename("Screenshot", ".bmp");

    if (fileio::write_bmp(filename.c_str(), ctx.pixels, ctx.width, ctx.height) != 0)
    {
        printf("Failed to write file: %s", filename.c_str());
        return -1;
    }

    free(ctx.pixels);

    printf("Wrote file: %s \n", filename.c_str());
}
