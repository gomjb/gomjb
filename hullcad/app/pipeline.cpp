#include "pipeline.h"

#include <algorithm>
#include <chrono>
#include <exception>

#include "hullcad/compute_backend.hpp"
#include "hullcad/lines_plan.hpp"
#include "hullcad/parallel.hpp"

namespace hullcad::app {

namespace {

double seconds_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

ShipRef ship_ref(const TankerParams& p) { return {p.Lpp, p.B, 0.5 * p.Lpp, 1.025}; }

std::vector<double> gz_heels() {
    std::vector<double> h;
    for (double a = 0; a <= 70.0 + 1e-9; a += 2.5) h.push_back(a);
    return h;
}

}  // namespace

HullRun run_hull(const TankerParams& p, double KG, int threads) {
    HullRun r;
    r.params = p;
    r.KG = KG;
    const auto t0 = std::chrono::steady_clock::now();
    try {
        BSplineSurface S = build_hull_surface(p);
        r.mesh = tessellate_hull(S, 240, 80);
        auto be = make_cpu_backend(threads);
        be->upload(r.mesh);
        const ShipRef ref = ship_ref(p);
        r.design = hydrostatic_table(*be, ref, {p.T})[0];
        std::vector<double> drafts;
        for (double T = 1.0; T <= p.D - 1.0 + 1e-9; T += 0.5) drafts.push_back(T);
        r.table = hydrostatic_table(*be, ref, drafts);
        r.gz = gz_curve(*be, ref, r.design.volume, Vec3{r.design.B.x, 0.0, KG}, gz_heels());

        LinesPlanSpec spec{p.Lpp, p.B, p.D, p.T, {}, {}, {}};
        for (int i = 0; i <= 20; ++i) spec.stations.push_back(std::clamp(p.Lpp * i / 20.0, 0.05, p.Lpp - 0.05));
        for (double z = 1.0; z < p.D - 0.5; z += 2.0) spec.waterlines.push_back(z);
        for (double y = 4.0; y < 0.5 * p.B; y += 4.0) spec.buttocks.push_back(y);
        r.lines_svg = lines_plan_svg(r.mesh, spec);
    } catch (const std::exception& e) {
        r.error = e.what();
    }
    r.seconds = seconds_since(t0);
    return r;
}

std::vector<GZPoint> run_gz(const HullRun& hull, double KG, int threads) {
    auto be = make_cpu_backend(threads);
    be->upload(hull.mesh);
    return gz_curve(*be, ship_ref(hull.params), hull.design.volume, Vec3{hull.design.B.x, 0.0, KG}, gz_heels());
}

DivisionRun run_division(const HullRun& hull, const YardProfile& yard, const std::vector<double>& manual_frames,
                         int threads) {
    DivisionRun r;
    r.manual = !manual_frames.empty();
    const auto t0 = std::chrono::steady_clock::now();
    try {
        const TankerParams& p = hull.params;
        TankerStructure st = default_tanker_structure(p);
        ThreadPool pool(threads);
        WeightModel wm = build_weight_model(hull.mesh, p, st, default_cargo_zones(p, st, yard),
                                            default_end_zones(p, yard), 0.25, pool);
        r.steel_total = wm.ring_mass(0.0, p.Lpp);
        if (r.manual) {
            std::vector<double> xs;
            for (double f : manual_frames) xs.push_back(st.frames.x(f));
            r.division = evaluate_division(wm, p, st, xs, yard);
        } else {
            r.division = auto_divide(wm, p, st, yard);
        }
        r.summary = summarize_checks(r.division);
        if (!r.division.butts.empty()) r.svg = division_svg(r.division, hull.mesh, p, st, wm, yard);
    } catch (const std::exception& e) {
        r.error = e.what();
    }
    r.seconds = seconds_since(t0);
    return r;
}

}  // namespace hullcad::app
