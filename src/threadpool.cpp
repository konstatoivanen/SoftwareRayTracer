#include "pch.h"
#include "threadpool.h"

namespace sr::utilities
{
    threadpool::threadpool()
    {
        auto count = std::thread::hardware_concurrency();
        m_threads.reserve(count);

        for (auto i = 0u; i < count; ++i)
        {
            m_threads.emplace_back(std::bind(&threadpool::thread_entry, this, i));
        }
    }

    threadpool::~threadpool()
    {
        {
            std::unique_lock<std::mutex>l(m_lock);
            m_shutdown = true;
            m_conditional.notify_all();
        }

        for (auto& thread : m_threads)
        {
            thread.join();
        }
    }

    void threadpool::queue_job(std::function<void(void)> func)
    {
        std::unique_lock <std::mutex> l(m_lock);
        m_jobs.emplace(std::move(func));
        m_conditional.notify_one();
    }

    void threadpool::thread_entry(uint32_t i)
    {
        std::function <void(void)> job;
    
        while (1)
        {
            {
                std::unique_lock<std::mutex>l(m_lock);
    
                while (!m_shutdown && m_jobs.empty())
                {
                    m_conditional.wait(l);
                }
    
                if (m_jobs.empty())
                {
                    return;
                }
    
                job = std::move(m_jobs.front());
                m_jobs.pop();
                printf("Jobs remaining: %i   \r", (uint32_t)m_jobs.size());
            }
    
            job();
        }
    }
}