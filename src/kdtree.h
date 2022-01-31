#pragma once
#include "memoryblock.h"
#include "structs.h"
#include <cstdint>

namespace sr::utilities
{
    // Sources: 
    // https://dcgi.fel.cvut.cz/home/havran/ARTICLES/cgf2011.pdf
    // https://graphics.cg.uni-saarland.de/courses/cg1-2017/slides/CG03-SpatialIndex.pdf
    // https://www.keithlantz.net/2013/04/kd-tree-construction-using-the-surface-area-heuristic-stack-based-traversal-and-the-hyperplane-separation-theorem/
    // https://medium.com/@bromanz/how-to-create-awesome-accelerators-the-surface-area-heuristic-e14b5dec6160

    struct facerange
    {
        uint32_t firstIndex = 0u;
        uint32_t indexCount = 0u;
    };

    struct raycasthit
    {
        int32_t index = -1;
        float distance = 1.0e+38f;
        float uv[2]{};
    };

    class kdtree
    {
        public:
            kdtree(const structs::mesh* mesh);
            kdtree(kdtree const&) = delete;
            kdtree& operator=(kdtree const&) = delete;

            bool raycast(const float* origin, const float* direction, raycasthit* hit) const;
        
            constexpr const structs::mesh* get_mesh() const { return m_mesh; }
            constexpr const uint32_t get_node_count() const { return m_nodeCount; }
            constexpr const uint32_t get_depth() const { return m_depth; }

        private:
            constexpr static uint32_t LEAF_FLAG = 4;
            constexpr static uint32_t FLAG_MASK_OFFS = 28;
            constexpr static uint32_t FLAG_MASK = 0xFu;
            constexpr static uint32_t INDEX_MASK = 0xFFFFFFu;
            constexpr static uint32_t MIN_SPLIT_FACES = 2;
            
            struct kdnode
            {
                uint32_t data;
                float offset;
            };
            
            bool compute_split(const math::bounds& bounds, uint32_t* faces, uint32_t faceCount, float costThreshold, float* offset, uint32_t* axis);
            void build_recursive(uint32_t nodeIndex, const math::bounds& bounds, uint32_t* faces, uint32_t faceCount, uint32_t depth);

            const structs::mesh* m_mesh = nullptr;
            memoryBlock<kdnode> m_nodes;
            memoryBlock<uint32_t> m_faces;
            memoryBlock<facerange> m_faceRanges;
            uint32_t m_depth = 0u;
            uint32_t m_nodeCount = 1u;
            uint32_t m_faceCount = 0u;
            uint32_t m_faceRangeCount = 0u;
    };
}