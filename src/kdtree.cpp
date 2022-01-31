#include "pch.h"
#include "kdtree.h"
#include "math.h"
#include <stack>

// Sources: 
// https://dcgi.fel.cvut.cz/home/havran/ARTICLES/cgf2011.pdf
// https://graphics.cg.uni-saarland.de/courses/cg1-2017/slides/CG03-SpatialIndex.pdf
// https://www.keithlantz.net/2013/04/kd-tree-construction-using-the-surface-area-heuristic-stack-based-traversal-and-the-hyperplane-separation-theorem/
// https://medium.com/@bromanz/how-to-create-awesome-accelerators-the-surface-area-heuristic-e14b5dec6160

namespace sr::utilities
{
    using namespace sr::math;
    using namespace sr::structs;

    struct splitplane
    {
        constexpr static const uint32_t FLAG_START = 0u;
        constexpr static const uint32_t FLAG_END = 1u;

        float offset;
        uint32_t type;
        constexpr bool operator < (const splitplane& other) { return offset < other.offset; }
    };

    struct stacknode
    {
        uint32_t index;
        float tmin;
        float tmax;
    };

    kdtree::kdtree(const structs::mesh* mesh) :
        m_nodes(mesh->indexCount / 6),
        m_faceRanges(mesh->indexCount / 6),
        m_faces(mesh->indexCount),
        m_mesh(mesh)
    {
        std::vector<uint32_t> faces;
        faces.resize(mesh->indexCount / 3);

        for (auto i = 0u; i < faces.size(); ++i)
        {
            faces[i] = i;
        }

        build_recursive(0u, mesh->bounds, faces.data(), (uint32_t)faces.size(), 1u);
    }
    
    bool kdtree::raycast(const float* origin, const float* direction, raycasthit* hit) const
    {
        float tmin, tmax;

        if (!intersect_bounds(m_mesh->bounds, origin, direction, &tmin, &tmax))
        {
            return false;
        }

        memoryBlock<stacknode> stack(1 + m_nodeCount / 6);
        auto stackSize = 1u;
        stack[0u] = { 0u, tmin, tmax };

        while (stackSize > 0u)
        {
            auto& entry = stack[--stackSize];
            auto nodeIndex = entry.index;
            tmin = entry.tmin;
            tmax = entry.tmax;

            auto node = m_nodes.get_offset(nodeIndex);
            auto flag = (node->data >> FLAG_MASK_OFFS) & FLAG_MASK;
            auto index = node->data & INDEX_MASK;

            while (flag != LEAF_FLAG)
            {
                auto tsplit = (node->offset - origin[flag]) / direction[flag];
                auto first = index;
                auto second = index + 1;

                if (node->offset - origin[flag] < 0.0f)
                {
                    first = index + 1;
                    second = index;
                }

                if (tsplit >= tmax || tsplit < 0.0f)
                {
                    nodeIndex = first;
                }
                else if (tsplit <= tmin)
                {
                    nodeIndex = second;
                }
                else
                {
                    stack.validate(stackSize + 1);
                    stack[stackSize++] = { second, tsplit, tmax };
                    nodeIndex = first;
                    tmax = tsplit;
                }

                node = m_nodes.get_offset(nodeIndex);
                flag = (node->data >> FLAG_MASK_OFFS) & FLAG_MASK;
                index = node->data & INDEX_MASK;
            }

            auto range = m_faceRanges.get_offset(index);
            auto positions = m_mesh->vertexPositions;
            auto indices = m_mesh->indices;
            auto faceIndices = m_faces.get_offset(range->firstIndex);
            auto distance = 0.0f;
            float barycoords[2];
            hit->index = -1;
            hit->distance = tmax;

            for (auto i = 0u; i < range->indexCount; ++i)
            {
                if (raycast_triangle(origin, direction,
                    positions + indices[faceIndices[i] * 3 + 0] * 3,
                    positions + indices[faceIndices[i] * 3 + 1] * 3,
                    positions + indices[faceIndices[i] * 3 + 2] * 3,
                    &distance,
                    barycoords) && distance < hit->distance)
                {
                    hit->distance = distance;
                    hit->index = faceIndices[i] * 3;
                    hit->uv[0] = barycoords[0];
                    hit->uv[1] = barycoords[1];
                }
            }

            if (hit->index != -1)
            {
                return true;
            }
        }

        return false;
    }

    bool kdtree::compute_split(const bounds& bounds, uint32_t* faces, uint32_t faceCount, float costThreshold, float* offset, uint32_t* axis)
    {
        // Very performance inoptimal. (depending on the compiler) could cause big allocs on every node :/
        std::vector<splitplane> planes[3]{};
        auto size = bounds.bmax - bounds.bmin;
        auto area = 2.0f * (size.x * size.y + size.y * size.z + size.z * size.x);
        auto minCost = 1.0e+38f;

        const float costIntersect = 1.0f;
        const float costTraversal = 1.0f;
        
        for (auto i = 0u; i < faceCount; ++i)
        {
            auto tri = m_mesh->indices + faces[i] * 3;

            planes[0].push_back({ fmin(fmin(m_mesh->vertexPositions[tri[0] * 3 + 0], m_mesh->vertexPositions[tri[1] * 3 + 0]), m_mesh->vertexPositions[tri[2] * 3 + 0]), splitplane::FLAG_START });
            planes[1].push_back({ fmin(fmin(m_mesh->vertexPositions[tri[0] * 3 + 1], m_mesh->vertexPositions[tri[1] * 3 + 1]), m_mesh->vertexPositions[tri[2] * 3 + 1]), splitplane::FLAG_START });
            planes[2].push_back({ fmin(fmin(m_mesh->vertexPositions[tri[0] * 3 + 2], m_mesh->vertexPositions[tri[1] * 3 + 2]), m_mesh->vertexPositions[tri[2] * 3 + 2]), splitplane::FLAG_START });
            planes[0].push_back({ fmax(fmax(m_mesh->vertexPositions[tri[0] * 3 + 0], m_mesh->vertexPositions[tri[1] * 3 + 0]), m_mesh->vertexPositions[tri[2] * 3 + 0]), splitplane::FLAG_END });
            planes[1].push_back({ fmax(fmax(m_mesh->vertexPositions[tri[0] * 3 + 1], m_mesh->vertexPositions[tri[1] * 3 + 1]), m_mesh->vertexPositions[tri[2] * 3 + 1]), splitplane::FLAG_END });
            planes[2].push_back({ fmax(fmax(m_mesh->vertexPositions[tri[0] * 3 + 2], m_mesh->vertexPositions[tri[1] * 3 + 2]), m_mesh->vertexPositions[tri[2] * 3 + 2]), splitplane::FLAG_END });
        }

        std::sort(planes[0].begin(), planes[0].end());
        std::sort(planes[1].begin(), planes[1].end());
        std::sort(planes[2].begin(), planes[2].end());

        for (auto i = 0u; i < 3; ++i)
        {
            uint32_t countL = 0u, countR = faceCount, flag = 0u;

            for (auto j = 0u; j < faceCount * 2u; ++j)
            {
                if (planes[i][j].type == splitplane::FLAG_START)
                {
                    if (flag)
                    {
                        countR--;
                        flag = 0u;
                    }

                    countL++;
                }
                else if (planes[i][j].type == splitplane::FLAG_END)
                {
                    if (flag)
                    {
                        countR--;
                    }

                    flag = 1u;
                }

                auto planeOffset = planes[i][j].offset;

                if (planeOffset <= bounds.bmin[i] || planeOffset >= bounds.bmax[i])
                {
                    continue;
                }

                auto dl = planeOffset - bounds.bmin[i];
                auto dr = bounds.bmax[i] - planeOffset;
                auto v0 = size[(i + 1) % 3];
                auto v1 = size[(i + 2) % 3];

                auto areaL = 2.0f * (v0 * v1 + v0 * dl + v1 * dl);
                auto areaR = 2.0f * (v0 * v1 + v0 * dr + v1 * dr);
                
                // Surface area heuristic
                auto cost = costTraversal + (areaL / area) * countL * costIntersect + (areaR / area) * countR * costIntersect;

                if (cost < minCost)
                {
                    minCost = cost;
                    *offset = planeOffset;
                    *axis = i;
                }
            }
        }

        return minCost > costThreshold;
    }
    
    void kdtree::build_recursive(uint32_t nodeIndex, const bounds& bounds, uint32_t* faces, uint32_t faceCount, uint32_t depth)
    {
        if (depth > m_depth)
        {
            m_depth = depth;
        }

        auto offset = 0.0f;
        auto axis = 0u;

        if (faceCount <= MIN_SPLIT_FACES || !compute_split(bounds, faces, faceCount, 16.0f, &offset, &axis))
        {
            auto leaf = m_nodes.get_offset(nodeIndex);

            m_faces.validate(m_faceCount + faceCount);
            memcpy(m_faces.get_offset(m_faceCount), faces, sizeof(uint32_t) * faceCount);

            m_faceRanges.validate(m_faceRangeCount + 1);
            m_faceRanges[m_faceRangeCount] = { m_faceCount, faceCount };
            leaf->data = (m_faceRangeCount & INDEX_MASK) | (LEAF_FLAG << FLAG_MASK_OFFS);
            leaf->offset = 0.0f;
            m_faceCount += faceCount;
            m_faceRangeCount++;
            return;
        }

        std::vector<uint32_t> facesRight;
        facesRight.reserve(faceCount);
        auto faceCountRight = 0u;

        for (int32_t i = faceCount - 1; i >= 0; --i)
        {
            auto face = faces[i];
            auto pos = m_mesh->vertexPositions;
            auto tri = m_mesh->indices;
            auto pmin = fmin(fmin(pos[tri[face * 3 + 0] * 3 + axis], pos[tri[face * 3 + 1] * 3 + axis]), pos[tri[face * 3 + 2] * 3 + axis]);
            auto pmax = fmax(fmax(pos[tri[face * 3 + 0] * 3 + axis], pos[tri[face * 3 + 1] * 3 + axis]), pos[tri[face * 3 + 2] * 3 + axis]);

            if (pmax > offset)
            {
                facesRight.push_back(face);
            }

            if (pmin > offset)
            {
                faces[i] = faces[--faceCount];
            }
        }

        math::bounds bbl = bounds, bbr = bounds;
        bbl.bmax[axis] = bbr.bmin[axis] = offset;

        m_nodeCount += 2;
        m_nodes.validate(m_nodeCount);
        auto firstChild = m_nodeCount - 2;

        auto node = m_nodes.get_offset(nodeIndex);
        node->data = (firstChild & INDEX_MASK) | (axis << FLAG_MASK_OFFS);
        node->offset = offset;
        build_recursive(firstChild, bbl, faces, faceCount, depth + 1u);
        build_recursive(firstChild + 1, bbr, facesRight.data(), (uint32_t)facesRight.size(), depth + 1u);
    }
}