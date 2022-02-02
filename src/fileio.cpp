#include "pch.h"
#include "fileio.h"
#include <tinyobjloader/tiny_obj_loader.h>
#include <filesystem>
#include <fstream>

namespace sr::utilities::fileio
{
    static std::string trim_string(const std::string& value)
    {
        auto first = value.find_first_not_of(" \n\r");

        if (first == std::string::npos)
        {
            return value;
        }

        return value.substr(first, (value.find_last_not_of(" \n\r") - first + 1));
    }


    std::string get_free_filename(const std::string& name, const std::string& extension)
    {
        auto index = 0;
        std::string filename; 

        do
        {
           filename = name + std::to_string(index++) + extension;
        }
        while (std::filesystem::exists(filename));

        return filename;
    }

    // Source: https://elcharolin.wordpress.com/2018/11/28/read-and-write-bmp-files-in-c-c/
    int write_bmp(const char* filepath, uint8_t* pixels, uint32_t width, uint32_t height)
    {
        const uint32_t BYTES_PER_PIXEL = 4u;
        const int32_t DATA_OFFSET_OFFSET = 0x000A;
        const int32_t WIDTH_OFFSET = 0x0012;
        const int32_t HEIGHT_OFFSET = 0x0016;
        const int32_t BITS_PER_PIXEL_OFFSET = 0x001C;
        const int32_t HEADER_SIZE = 14;
        const int32_t INFO_HEADER_SIZE = 40;
        const int32_t NO_COMPRESION = 0;
        const int32_t MAX_NUMBER_OF_COLORS = 0;
        const int32_t ALL_COLORS_REQUIRED = 0;

        FILE* outputFile = nullptr;

        #if _WIN32
            auto error = fopen_s(&outputFile, filepath, "wb");
    
            if (error != 0)
            {
                return -1;
            }
        #else
            file = fopen(filepath, "rb");
        #endif

        //*****HEADER************//
        const char* BM = "BM";
        fwrite(&BM[0], 1, 1, outputFile);
        fwrite(&BM[1], 1, 1, outputFile);
        int paddedRowSize = (int)(4 * ceil((float)width / 4.0f)) * BYTES_PER_PIXEL;
        uint32_t fileSize = paddedRowSize * height + HEADER_SIZE + INFO_HEADER_SIZE;
        fwrite(&fileSize, 4, 1, outputFile);
        uint32_t reserved = 0x0000;
        fwrite(&reserved, 4, 1, outputFile);
        uint32_t dataOffset = HEADER_SIZE + INFO_HEADER_SIZE;
        fwrite(&dataOffset, 4, 1, outputFile);

        //*******INFO*HEADER******//
        uint32_t infoHeaderSize = INFO_HEADER_SIZE;
        fwrite(&infoHeaderSize, 4, 1, outputFile);
        fwrite(&width, 4, 1, outputFile);
        fwrite(&height, 4, 1, outputFile);
        uint16_t planes = 1; //always 1
        fwrite(&planes, 2, 1, outputFile);
        uint16_t bitsPerPixel = BYTES_PER_PIXEL * 8;
        fwrite(&bitsPerPixel, 2, 1, outputFile);
        //write compression
        uint32_t compression = NO_COMPRESION;
        fwrite(&compression, 4, 1, outputFile);
        //write image size(in bytes)
        uint32_t imageSize = width * height * BYTES_PER_PIXEL;
        fwrite(&imageSize, 4, 1, outputFile);
        uint32_t resolutionX = 11811; //300 dpi
        uint32_t resolutionY = 11811; //300 dpi
        fwrite(&resolutionX, 4, 1, outputFile);
        fwrite(&resolutionY, 4, 1, outputFile);
        uint32_t colorsUsed = MAX_NUMBER_OF_COLORS;
        fwrite(&colorsUsed, 4, 1, outputFile);
        uint32_t importantColors = ALL_COLORS_REQUIRED;
        fwrite(&importantColors, 4, 1, outputFile);
        int32_t unpaddedRowSize = width * BYTES_PER_PIXEL;

        for (uint32_t y = 0u; y < height; ++y)
        for (uint32_t x = 0u; x < width; ++x)
        {
            uint32_t index = (x + y * width) * BYTES_PER_PIXEL;
            uint8_t color[4] = { pixels[index + 2], pixels[index + 1], pixels[index + 0], pixels[index + 3] };
            fwrite(color, sizeof(uint8_t), 4, outputFile);
        }

        return fclose(outputFile);
    }

    int load_config(const char* filepath, sr::structs::config* cfg)
    {
        if (cfg == nullptr)
        {
            printf("Cannot read config into a null pointer!");
            return -1;
        }

        std::ifstream file(filepath, std::ios::in);

        if (!file)
        {
            printf("Failed to open config!");
            return -1;
        }

        std::string result;
        std::string lineBuffer;
        std::map<std::string, std::string> valuemap;

        while (std::getline(file, lineBuffer))
        {
            auto split = lineBuffer.find(':');

            if (split != std::string::npos)
            {
                auto first = trim_string(lineBuffer.substr(0, split));
                auto second = trim_string(lineBuffer.substr(split + 1));
                valuemap[first] = second;
            }
        }

        auto iter = valuemap.find("meshPath");
        cfg->meshPath = iter != valuemap.end() ? iter->second : "";

        iter = valuemap.find("width");
        cfg->width = iter != valuemap.end() ? std::stoul(iter->second) : 256u;

        iter = valuemap.find("height");
        cfg->height = iter != valuemap.end() ? std::stoul(iter->second) : 256u;

        iter = valuemap.find("warpSize");
        cfg->warpSize = iter != valuemap.end() ? std::stoul(iter->second) : 4u;

        iter = valuemap.find("samples");
        cfg->samples = iter != valuemap.end() ? std::stoul(iter->second) : 1024u;

        iter = valuemap.find("bounces");
        cfg->bounces = iter != valuemap.end() ? std::stoul(iter->second) : 2u;

        iter = valuemap.find("mode");

        if (iter != valuemap.end())
        {
            if (strcmp(iter->second.c_str(), structs::SR_TRACE_MODE_STR_GGX) == 0)
            {
                cfg->mode = structs::SR_TRACE_MODE_GGX;
            }
            else if (strcmp(iter->second.c_str(), structs::SR_TRACE_MODE_STR_RANDOM) == 0)
            {
                cfg->mode = structs::SR_TRACE_MODE_RANDOM;
            }
            else if (strcmp(iter->second.c_str(), structs::SR_TRACE_MODE_STR_ALBEDO) == 0)
            {
                cfg->mode = structs::SR_TRACE_MODE_ALBEDO;
            }
            else if (strcmp(iter->second.c_str(), structs::SR_TRACE_MODE_STR_NORMALS) == 0)
            {
                cfg->mode = structs::SR_TRACE_MODE_NORMALS;
            }
            else if (strcmp(iter->second.c_str(), structs::SR_TRACE_MODE_STR_EMISSION) == 0)
            {
                cfg->mode = structs::SR_TRACE_MODE_EMISSION;
            }
        }

        return 0;
    }

    int load_mesh(const char* filepath, sr::structs::mesh* mesh, float boundsPadding)
    {
        if (mesh == nullptr)
        {
            printf("Cannot read mesh into a null pointer!");
            return -1;
        }

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string err;

        auto directory = std::filesystem::path(filepath).remove_filename().string();

        bool success = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filepath, directory.c_str(), true);

        if (!err.empty())
        {
            printf(err.c_str());
            return -1;
        }

        if (!success)
        {
            printf("Failed to load OBJ");
            return -1;
        }

        if (materials.empty())
        {
            printf("OBJ doesn't any materials!");
            return -1;
        }

        if (attrib.normals.empty() || attrib.vertices.empty())
        {
            printf("OBJ doesn't have the required vertex attirbutes (position & normal)!");
            return -1;
        }

        auto invertices = attrib.vertices.data();
        auto innormals = attrib.normals.data();

        std::map<structs::indexset, uint32_t> indexmap;
        std::vector<uint32_t> indices;
        std::vector<float> vertexPositions;
        std::vector<float> vertexNormals;
        std::vector<float> vertexAlbedo;
        std::vector<float> vertexEmission;
        auto index = 0u;

        mesh->bounds.bmax[0] = mesh->bounds.bmax[1] = mesh->bounds.bmax[2] = -3.4028237e+37f;
        mesh->bounds.bmin[0] = mesh->bounds.bmin[1] = mesh->bounds.bmin[2] = 3.4028237e+37f;

        for (size_t i = 0; i < shapes.size(); ++i)
        {
            auto& tris = shapes.at(i).mesh.indices;
            auto& materialIds = shapes.at(i).mesh.material_ids;
            auto tcount = (uint32_t)tris.size();

            for (uint32_t j = 0; j < tcount; ++j)
            {
                auto& tri = tris.at(j);
                structs::indexset triKey = { (uint32_t)tri.vertex_index, (uint32_t)tri.normal_index, (uint32_t)tri.texcoord_index };

                auto iter = indexmap.find(triKey);

                if (iter != indexmap.end())
                {
                    indices.push_back(iter->second);
                    continue;
                }

                indices.push_back(index);
                indexmap[triKey] = index++;
                
                auto materialId = materialIds[j / 3];
                auto albedo = materials[materialId].diffuse;
                auto emission = materials[materialId].emission;

                for (uint32_t k = 0u; k < 3; ++k)
                {
                    auto v = invertices + tri.vertex_index * 3;
                    auto n = innormals + tri.normal_index * 3;

                    vertexPositions.push_back(v[k]);
                    vertexNormals.push_back(n[k]);
                    vertexAlbedo.push_back(albedo[k]);
                    vertexEmission.push_back(emission[k]);

                    if (mesh->bounds.bmax[k] < v[k])
                    {
                        mesh->bounds.bmax[k] = v[k];
                    }

                    if (mesh->bounds.bmin[k] > v[k])
                    {
                        mesh->bounds.bmin[k] = v[k];
                    }
                }
            }
        }

        for (auto i = 0u; i < 3; ++i)
        {
            mesh->bounds.bmin[i] -= boundsPadding;
            mesh->bounds.bmax[i] += boundsPadding;
        }

        mesh->vertexCount = vertexPositions.size() / 3;
        mesh->indexCount = indices.size();

        mesh->vertexPositions = reinterpret_cast<float*>(malloc(sizeof(float) * vertexPositions.size()));
        mesh->vertexNormals = reinterpret_cast<float*>(malloc(sizeof(float) * vertexNormals.size()));
        mesh->vertexAlbedo = reinterpret_cast<float*>(malloc(sizeof(float) * vertexAlbedo.size()));
        mesh->vertexEmission = reinterpret_cast<float*>(malloc(sizeof(float) * vertexEmission.size()));
        mesh->indices = reinterpret_cast<uint32_t*>(malloc(sizeof(uint32_t) * indices.size()));

        memcpy(mesh->vertexPositions, vertexPositions.data(), sizeof(float) * vertexPositions.size());
        memcpy(mesh->vertexNormals, vertexNormals.data(), sizeof(float) * vertexNormals.size());
        memcpy(mesh->vertexAlbedo, vertexAlbedo.data(), sizeof(float) * vertexAlbedo.size());
        memcpy(mesh->vertexEmission, vertexEmission.data(), sizeof(float) * vertexEmission.size());
        memcpy(mesh->indices, indices.data(), sizeof(uint32_t) * indices.size());
    
        return 0u;
    }

    void unload_mesh(sr::structs::mesh* mesh)
    {
        if (mesh != nullptr)
        {
            free(mesh->indices);
            free(mesh->vertexPositions);
            free(mesh->vertexNormals);
            free(mesh->vertexAlbedo);
            free(mesh->vertexEmission);
        }
    }
}