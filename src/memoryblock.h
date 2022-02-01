#pragma once
#include <exception>
#include <iostream>

namespace sr::utilities
{
    template<typename T>
    class memoryblock
    {
        public:
            memoryblock(size_t count)
            {
                validate(count);
            }

            ~memoryblock()
            {
                if (m_data != nullptr)
                {
                    free(m_data);
                }
            }

            memoryblock(memoryblock const&) = delete;
            memoryblock& operator=(memoryblock const&) = delete;

            void validate(size_t count, bool discard = false)
            {
                if (count <= m_count)
                {
                    return;
                }

                auto newCount = m_count == 0 ? count : m_count;

                while (newCount < count)
                {
                    newCount <<= 1;
                }

                auto newbuffer = calloc(newCount, sizeof(T));

                if (newbuffer == nullptr)
                {
                    throw std::runtime_error("Failed to allocate a new buffer!");
                }

                if (m_data != nullptr)
                {
                    if (!discard)
                    {
                        memcpy(newbuffer, m_data, sizeof(T) * m_count);
                    }

                    free(m_data);
                }

                m_data = newbuffer;
                m_count = newCount;
            }

            void clear() { memset(m_data, 0, sizeof(T) * m_count); }

            T* get_offset(size_t offset) { return reinterpret_cast<T*>(m_data) + offset; }
            T const* get_offset(size_t offset) const { return reinterpret_cast<const T*>(m_data) + offset; }

            T* get_data() { return reinterpret_cast<T*>(m_data); }
            T const* get_data() const { return reinterpret_cast<const T*>(m_data); }

            T& operator [](size_t i) { return reinterpret_cast<T*>(m_data)[i]; }
            T const& operator [](size_t i) const { return reinterpret_cast<const T*>(m_data)[i]; }

            operator T* () { return get_data(); }
            operator T const* () const { return get_data(); }

            constexpr size_t get_count() const { return m_count; }
            constexpr size_t get_size() const { return m_count * sizeof(T); }

        private:
            void* m_data = nullptr;
            size_t m_count = 0ull;
    };
}