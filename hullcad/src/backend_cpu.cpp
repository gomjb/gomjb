#include <algorithm>
#include <stdexcept>
#include <vector>

#include "hullcad/compute_backend.hpp"
#include "hullcad/parallel.hpp"

namespace hullcad {

namespace {

constexpr size_t kChunk = 2048;  // triangles per partial sum; fixes the summation order

class CpuBackend final : public ComputeBackend {
public:
    explicit CpuBackend(int threads) : pool_(threads) {}

    std::string name() const override { return "cpu(" + std::to_string(pool_.size()) + " threads)"; }

    void upload(const TriMesh& m) override {
        tris_.clear();
        tris_.reserve(m.tris.size() * 3);
        for (auto& t : m.tris)
            for (int k = 0; k < 3; ++k) tris_.push_back(m.verts[t[k]]);  // flat triangle soup
        nchunks_ = (m.tris.size() + kChunk - 1) / kChunk;
    }

    void evaluate(std::span<const WaterPlane> planes, std::span<HydroAccum> out) override {
        if (out.size() != planes.size()) throw std::invalid_argument("evaluate: size mismatch");
        const size_t np = planes.size();
        if (np >= static_cast<size_t>(pool_.size())) {
            // Enough conditions to keep every thread busy: one condition per task.
            pool_.parallel_for(np, [&](size_t i) { out[i] = eval_plane(planes[i]); });
        } else {
            // Few conditions: split each condition's triangles across threads.
            std::vector<HydroAccum> partial(np * nchunks_);
            pool_.parallel_for(np * nchunks_, [&](size_t k) {
                partial[k] = eval_chunk(planes[k / nchunks_], k % nchunks_);
            });
            for (size_t i = 0; i < np; ++i) {
                HydroAccum acc;
                for (size_t c = 0; c < nchunks_; ++c) accum_add(acc, partial[i * nchunks_ + c]);
                out[i] = acc;
            }
        }
    }

private:
    HydroAccum eval_chunk(const WaterPlane& wp, size_t chunk) const {
        HydroAccum acc;
        const size_t ntri = tris_.size() / 3;
        const size_t end = std::min(ntri, (chunk + 1) * kChunk);
        for (size_t t = chunk * kChunk; t < end; ++t)
            accumulate_triangle(tris_[3 * t], tris_[3 * t + 1], tris_[3 * t + 2], wp, acc);
        return acc;
    }
    HydroAccum eval_plane(const WaterPlane& wp) const {
        HydroAccum acc;
        for (size_t c = 0; c < nchunks_; ++c) accum_add(acc, eval_chunk(wp, c));
        return acc;
    }

    ThreadPool pool_;
    std::vector<Vec3> tris_;
    size_t nchunks_ = 0;
};

}  // namespace

std::unique_ptr<ComputeBackend> make_cpu_backend(int threads) {
    return std::make_unique<CpuBackend>(threads);
}

#ifndef HULLCAD_HAVE_CUDA
std::unique_ptr<ComputeBackend> make_cuda_backend() { return nullptr; }
#endif

}  // namespace hullcad
