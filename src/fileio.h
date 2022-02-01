#pragma once
#include "Structs.h"
#include <cstdint>

namespace sr::utilities::fileio
{
    std::string get_free_filename(const std::string& name, const std::string& extension);
    int write_bmp(const char* filepath, uint8_t* pixels, uint32_t width, uint32_t height);
    int load_config(const char* filepath, sr::structs::config* cfg);
    int load_mesh(const char* filepath, sr::structs::mesh* mesh, float boundsPadding);
    void unload_mesh(sr::structs::mesh* mesh);
}