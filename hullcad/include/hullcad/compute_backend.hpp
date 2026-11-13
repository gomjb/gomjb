#pragma once
// Compute backend abstraction: evaluate many waterplanes against one hull mesh in a batch.
// Solvers (hydrostatic tables, GZ curves, later damage cases) only talk to this interface,
// so the same solver code runs on the multithreaded CPU backend or a GPU backend.
#include <memory>
#include <span>
#include <string>

#include "hullcad/hydro_kernel.hpp"
#include "hullcad/mesh.hpp"

namespace hullcad {

class ComputeBackend {
public:
    virtual ~ComputeBackend() = default;
    virtual std::string name() const = 0;
    virtual void upload(const TriMesh& mesh) = 0;
    // out[i] = integrals of the uploaded mesh against planes[i].
    virtual void evaluate(std::span<const WaterPlane> planes, std::span<HydroAccum> out) = 0;
};

// Multithreaded CPU backend. threads <= 0 uses all hardware threads.
// Results are bit-identical for any thread count and any batch size: triangles are summed in
// fixed-size chunks, and chunk partials are added in a fixed order.
std::unique_ptr<ComputeBackend> make_cpu_backend(int threads = 0);

// CUDA backend; returns nullptr when hullcad was built without HULLCAD_ENABLE_CUDA or no
// CUDA device is present. GPU sums use a different reduction order, so compare against the
// CPU backend with a relative tolerance, not bit-for-bit.
std::unique_ptr<ComputeBackend> make_cuda_backend();

}  // namespace hullcad
