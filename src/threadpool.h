#pragma once
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

namespace sr::utilities
{
    // Source: https://stackoverflow.com/questions/26516683/reusing-thread-in-loop-c
    class threadpool
    {
        public:
            threadpool();
            ~threadpool();
            threadpool(threadpool const&) = delete;
            threadpool& operator=(threadpool const&) = delete;

            void queue_job(std::function<void(void)> func);
            void wait_all(uint64_t queryInterval = 0ul);

        private:
            void thread_entry(uint32_t i);

            std::mutex m_lock;
            std::condition_variable m_conditional;
            std::queue<std::function<void(void)>> m_jobs;
            std::vector<std::thread> m_threads;
            std::atomic<uint32_t> m_idleCount;
            bool m_shutdown = false;
    };
}