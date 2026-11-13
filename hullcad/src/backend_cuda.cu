// CUDA backend: one thread block per waterplane, threads stride over triangles, then a
// fixed-shape shared-memory tree reduction. The per-triangle math is the same
// accumulate_triangle() used by the CPU backend (hydro_kernel.hpp).
//
// Note: consumer GeForce GPUs run FP64 at 1/32-1/64 of FP32 speed. Hydrostatics for
// approval-grade calculations should stay in double; data-centre GPUs (A100/H100) or
// workstation cards with full FP64 are the intended target for large batches.
#include <cuda_runtime.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "hullcad/compute_backend.hpp"

namespace hullcad {

namespace {

constexpr int kThreads = 256;

void check(cudaError_t e, const char* what) {
    if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}

__global__ void hydro_kernel(const Vec3* __restrict__ tris, int ntri, const WaterPlane* __restrict__ planes,
                             HydroAccum* __restrict__ out) {
    // Raw storage: __shared__ variables cannot have constructors (HydroAccum zero-initialises).
    __shared__ alignas(alignof(HydroAccum)) unsigned char sh_raw[kThreads * sizeof(HydroAccum)];
    HydroAccum* sh = reinterpret_cast<HydroAccum*>(sh_raw);
    const WaterPlane wp = planes[blockIdx.x];
    HydroAccum acc;
    for (int t = threadIdx.x; t < ntri; t += blockDim.x)
        accumulate_triangle(tris[3 * t], tris[3 * t + 1], tris[3 * t + 2], wp, acc);
    sh[threadIdx.x] = acc;
    __syncthreads();
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (threadIdx.x < s) accum_add(sh[threadIdx.x], sh[threadIdx.x + s]);
        __syncthreads();
    }
    if (threadIdx.x == 0) out[blockIdx.x] = sh[0];
}

class CudaBackend final : public ComputeBackend {
public:
    CudaBackend() {
        cudaDeviceProp prop{};
        check(cudaGetDeviceProperties(&prop, 0), "cudaGetDeviceProperties");
        name_ = std::string("cuda(") + prop.name + ")";
    }
    ~CudaBackend() override {
        cudaFree(d_tris_);
        cudaFree(d_planes_);
        cudaFree(d_out_);
    }
    std::string name() const override { return name_; }

    void upload(const TriMesh& m) override {
        std::vector<Vec3> soup;
        soup.reserve(m.tris.size() * 3);
        for (auto& t : m.tris)
            for (int k = 0; k < 3; ++k) soup.push_back(m.verts[t[k]]);
        cudaFree(d_tris_);
        d_tris_ = nullptr;
        check(cudaMalloc(&d_tris_, soup.size() * sizeof(Vec3)), "cudaMalloc tris");
        check(cudaMemcpy(d_tris_, soup.data(), soup.size() * sizeof(Vec3), cudaMemcpyHostToDevice), "upload tris");
        ntri_ = static_cast<int>(m.tris.size());
    }

    void evaluate(std::span<const WaterPlane> planes, std::span<HydroAccum> out) override {
        if (out.size() != planes.size()) throw std::invalid_argument("evaluate: size mismatch");
        const size_t n = planes.size();
        if (n == 0) return;
        if (n > cap_) {
            cudaFree(d_planes_);
            cudaFree(d_out_);
            check(cudaMalloc(&d_planes_, n * sizeof(WaterPlane)), "cudaMalloc planes");
            check(cudaMalloc(&d_out_, n * sizeof(HydroAccum)), "cudaMalloc out");
            cap_ = n;
        }
        check(cudaMemcpy(d_planes_, planes.data(), n * sizeof(WaterPlane), cudaMemcpyHostToDevice), "upload planes");
        hydro_kernel<<<static_cast<unsigned>(n), kThreads>>>(d_tris_, ntri_, d_planes_, d_out_);
        check(cudaGetLastError(), "hydro_kernel launch");
        check(cudaMemcpy(out.data(), d_out_, n * sizeof(HydroAccum), cudaMemcpyDeviceToHost), "download results");
    }

private:
    std::string name_;
    Vec3* d_tris_ = nullptr;
    WaterPlane* d_planes_ = nullptr;
    HydroAccum* d_out_ = nullptr;
    size_t cap_ = 0;
    int ntri_ = 0;
};

}  // namespace

std::unique_ptr<ComputeBackend> make_cuda_backend() {
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count == 0) return nullptr;
    return std::make_unique<CudaBackend>();
}

}  // namespace hullcad
