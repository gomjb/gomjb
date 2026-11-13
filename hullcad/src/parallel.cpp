#include "hullcad/parallel.hpp"

namespace hullcad {

ThreadPool::ThreadPool(int threads) {
    if (threads <= 0) threads = static_cast<int>(std::thread::hardware_concurrency());
    if (threads <= 0) threads = 1;
    for (int i = 0; i < threads - 1; ++i) workers_.emplace_back([this] { worker_loop(); });
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
    }
    cv_start_.notify_all();
    for (auto& w : workers_) w.join();
}

void ThreadPool::run_items() {
    for (;;) {
        size_t i = next_.fetch_add(1, std::memory_order_relaxed);
        if (i >= n_) break;
        try {
            (*job_)(i);
        } catch (...) {
            std::lock_guard<std::mutex> lk(mu_);
            if (!error_) error_ = std::current_exception();
            next_.store(n_);
        }
    }
}

void ThreadPool::worker_loop() {
    unsigned long seen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_start_.wait(lk, [&] { return stop_ || generation_ != seen; });
            if (stop_) return;
            seen = generation_;
        }
        run_items();
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (--active_ == 0) cv_done_.notify_one();
        }
    }
}

void ThreadPool::parallel_for(size_t n, const std::function<void(size_t)>& f) {
    if (n == 0) return;
    if (workers_.empty() || n == 1) {
        for (size_t i = 0; i < n; ++i) f(i);
        return;
    }
    {
        std::lock_guard<std::mutex> lk(mu_);
        job_ = &f;
        n_ = n;
        next_.store(0);
        active_ = static_cast<int>(workers_.size());
        error_ = nullptr;
        ++generation_;
    }
    cv_start_.notify_all();
    run_items();
    std::exception_ptr err;
    {
        std::unique_lock<std::mutex> lk(mu_);
        cv_done_.wait(lk, [&] { return active_ == 0; });
        job_ = nullptr;
        err = error_;
    }
    if (err) std::rethrow_exception(err);
}

}  // namespace hullcad
