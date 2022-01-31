#pragma once
#include "Structs.h"
#include <cstdint>

namespace sr::utilities::fileio
{
    void write_bmp(const char* filepath, unsigned char* pixels, uint32_t width, uint32_t height);
    int load_config(const char* filepath, sr::structs::config* cfg);
    int load_mesh(const char* filepath, sr::structs::mesh* mesh, float boundsPadding);
    void unload_mesh(sr::structs::mesh* mesh);
}