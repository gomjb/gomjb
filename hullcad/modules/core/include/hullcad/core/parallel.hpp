#pragma once
// Small persistent thread pool with a blocking parallel_for (dynamic scheduling).
// Results stay deterministic as long as each index writes only its own output.
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace hullcad {

class ThreadPool {
public:
    explicit ThreadPool(int threads = 0);
    ~ThreadPool();
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    int size() const { return static_cast<int>(workers_.size()) + 1; }  // + calling thread
    // Calls f(i) for i in [0, n). Blocks until all are done. Rethrows the first exception.
    void parallel_for(size_t n, const std::function<void(size_t)>& f);

private:
    void worker_loop();
    void run_items();

    std::vector<std::thread> workers_;
    std::mutex mu_;
    std::condition_variable cv_start_, cv_done_;
    const std::function<void(size_t)>* job_ = nullptr;
    size_t n_ = 0;
    std::atomic<size_t> next_{0};
    int active_ = 0;
    unsigned long generation_ = 0;
    bool stop_ = false;
    std::exception_ptr error_;
};

}  // namespace hullcad
