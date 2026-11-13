// 120k DWT tanker demo: parametric hull -> B-spline surface -> mesh -> hydrostatics, GZ,
// lines plan, and a CPU (and optional CUDA) throughput benchmark.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "hullcad/blocks.hpp"
#include "hullcad/compute_backend.hpp"
#include "hullcad/hull_param.hpp"
#include "hullcad/hydrostatics.hpp"
#include "hullcad/lines_plan.hpp"
#include "hullcad/mesh.hpp"

using namespace hullcad;

namespace {

double seconds_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

// Upright, heeled and trimmed grid: the shape of work a damage-stability or loading-condition
// sweep produces.
std::vector<WaterPlane> condition_grid(const TankerParams& p, int ndraft, int nheel, int ntrim) {
    std::vector<WaterPlane> g;
    for (int i = 0; i < ndraft; ++i)
        for (int j = 0; j < nheel; ++j)
            for (int k = 0; k < ntrim; ++k) {
                double T = 3.0 + (p.T + 2.0 - 3.0) * i / std::max(1, ndraft - 1);
                double heel = 60.0 * j / std::max(1, nheel - 1);
                double trim = -1.0 + 2.0 * k / std::max(1, ntrim - 1);
                g.push_back(make_plane(T, heel, trim, 0.5 * p.Lpp));
            }
    return g;
}

double bench(ComputeBackend& be, const std::vector<WaterPlane>& planes, int reps) {
    std::vector<HydroAccum> out(planes.size());
    be.evaluate(planes, out);  // warm-up
    auto t0 = std::chrono::steady_clock::now();
    for (int r = 0; r < reps; ++r) be.evaluate(planes, out);
    return seconds_since(t0) / reps;
}

}  // namespace

int main(int argc, char** argv) {
    std::string outdir = "out";
    int threads = 0;
    bool run_bench = true;
    DivisionRules rules;
    std::vector<double> manual_butt_frames;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) outdir = argv[++i];
        else if (a == "--threads" && i + 1 < argc) threads = std::atoi(argv[++i]);
        else if (a == "--no-bench") run_bench = false;
        else if (a == "--max-block" && i + 1 < argc) rules.max_block_weight = std::atof(argv[++i]);
        else if (a == "--ring-length" && i + 2 < argc) {
            rules.min_ring_length = std::atof(argv[++i]);
            rules.max_ring_length = std::atof(argv[++i]);
        } else if (a == "--butts" && i + 1 < argc) {
            std::stringstream ss(argv[++i]);
            for (std::string t; std::getline(ss, t, ',');) manual_butt_frames.push_back(std::atof(t.c_str()));
        } else {
            std::printf("usage: hullcad_demo [--out DIR] [--threads N] [--no-bench]\n"
                        "                    [--max-block TONNES] [--ring-length MIN MAX] [--butts FR,FR,...]\n");
            return 2;
        }
    }
    std::filesystem::create_directories(outdir);

    TankerParams p;
    ShipRef ref{p.Lpp, p.B, 0.5 * p.Lpp, 1.025};

    auto t0 = std::chrono::steady_clock::now();
    auto rows = offsets(p);
    BSplineSurface S = skin(rows, 3, 3);
    TriMesh mesh = tessellate_hull(S, 240, 80);
    std::printf("Hull surface: bicubic B-spline, %d x %d control points (from %zu stations x %d points)\n",
                S.nu, S.nv, rows.size(), p.points_per_section);
    std::printf("Mesh: %zu triangles, closed+consistent: %s  (built in %.3f s)\n", mesh.tris.size(),
                is_closed_and_consistent(mesh) ? "yes" : "NO", seconds_since(t0));

    auto cpu = make_cpu_backend(threads);
    cpu->upload(mesh);

    // Principal particulars at design draft.
    HydroResult d = hydrostatic_table(*cpu, ref, {p.T})[0];
    std::printf("\n== Principal particulars (design draft) ==\n");
    std::printf("Lpp %.1f m  B %.1f m  D %.1f m  T %.1f m\n", p.Lpp, p.B, p.D, p.T);
    std::printf("Displacement %.0f t  (volume %.0f m3)\n", d.displacement, d.volume);
    std::printf("Cb %.3f  Cw %.3f  LCB %.2f m fwd of midship  KB %.2f m\n", d.Cb, d.Cw, d.B.x - ref.x_ref, d.B.z);
    std::printf("Awp %.0f m2  LCF %.2f m fwd of midship  TPC %.1f t/cm  MCT %.0f t.m/cm\n", d.Awp,
                d.F.x - ref.x_ref, d.TPC, d.MCT);
    std::printf("BMt %.2f m  KMt %.2f m  BMl %.1f m\n", d.BM_T, d.KM_T, d.BM_L);

    // Offsets.
    {
        std::ofstream f(outdir + "/offsets.csv");
        f << "station,x,point,y,z\n";
        for (size_t i = 0; i < rows.size(); ++i)
            for (size_t j = 0; j < rows[i].size(); ++j)
                f << i << ',' << rows[i][j].x << ',' << j << ',' << rows[i][j].y << ',' << rows[i][j].z << '\n';
    }

    // Hydrostatic table.
    std::vector<double> drafts;
    for (double T = 1.0; T <= 18.0 + 1e-9; T += 0.5) drafts.push_back(T);
    auto tab = hydrostatic_table(*cpu, ref, drafts);
    {
        std::ofstream f(outdir + "/hydrostatics.csv");
        f << "draft_m,volume_m3,displacement_t,LCB_m,KB_m,Awp_m2,LCF_m,TPC_t_cm,MCT_tm_cm,BMt_m,KMt_m,BMl_m,KMl_m,Cb,Cw\n";
        for (auto& r : tab)
            f << r.draft << ',' << r.volume << ',' << r.displacement << ',' << r.B.x - ref.x_ref << ',' << r.B.z << ','
              << r.Awp << ',' << r.F.x - ref.x_ref << ',' << r.TPC << ',' << r.MCT << ',' << r.BM_T << ',' << r.KM_T
              << ',' << r.BM_L << ',' << r.KM_L << ',' << r.Cb << ',' << r.Cw << '\n';
    }
    std::printf("\nHydrostatic table: %zu drafts -> %s/hydrostatics.csv\n", tab.size(), outdir.c_str());

    // GZ curve, full-load condition with an ASSUMED KG (no loading condition yet).
    const double KG = 13.0;
    std::vector<double> heels;
    for (double h = 0; h <= 70.0 + 1e-9; h += 2.5) heels.push_back(h);
    auto gz = gz_curve(*cpu, ref, d.volume, Vec3{d.B.x, 0.0, KG}, heels);
    {
        std::ofstream f(outdir + "/gz.csv");
        f << "heel_deg,GZ_m,draft_m,iterations\n";
        for (auto& g : gz) f << g.heel << ',' << g.GZ << ',' << g.draft << ',' << g.iterations << '\n';
    }
    double gzmax = 0, at = 0;
    for (auto& g : gz) if (g.GZ > gzmax) { gzmax = g.GZ; at = g.heel; }
    std::printf("GZ curve (fixed trim, KG %.1f m assumed, GM %.2f m): GZmax %.2f m at %.1f deg -> %s/gz.csv\n", KG,
                d.KM_T - KG, gzmax, at, outdir.c_str());

    // Lines plan and mesh.
    LinesPlanSpec spec{p.Lpp, p.B, p.D, p.T, {}, {}, {}};
    for (int i = 0; i <= 20; ++i) spec.stations.push_back(std::clamp(p.Lpp * i / 20.0, 0.05, p.Lpp - 0.05));
    for (double z = 1.0; z <= 19.0; z += 2.0) spec.waterlines.push_back(z);
    for (double y = 4.0; y <= 20.0; y += 4.0) spec.buttocks.push_back(y);
    write_lines_plan_svg(mesh, spec, outdir + "/lines.svg");
    write_obj(mesh, outdir + "/hull.obj");
    std::printf("Lines plan -> %s/lines.svg, mesh -> %s/hull.obj\n", outdir.c_str(), outdir.c_str());

    // Block division: structure layout -> per-zone steel weights -> butts (auto or manual) -> checks.
    {
        TankerStructure st = default_tanker_structure(p);
        auto zones = default_cargo_zones(p, st);
        ThreadPool pool(threads);
        auto tw = std::chrono::steady_clock::now();
        WeightModel wm = build_weight_model(mesh, p, st, zones, 0.25, pool);
        double t_wm = seconds_since(tw);
        Mass total = wm.ring_mass(0.0, p.Lpp);
        Mass cargo = wm.ring_mass(st.cargo_x0(), st.cargo_x1());
        std::printf("\n== Block division ==\n");
        std::printf("Weight model: %d slices x %d zones, %zu transverse members, built in %.3f s (%d threads)\n",
                    wm.nslice, wm.nzone, wm.transverse.size(), t_wm, pool.size());
        std::printf("Estimated hull steel (modelled members only): %.0f t total, %.0f t in cargo region, LCG %.1f m\n",
                    total.w, cargo.w, total.cog().x);

        Division div;
        if (manual_butt_frames.empty()) {
            div = auto_divide(wm, zones, p, st, rules);
            std::printf("Automatic division (limit %.0f t, ring %.0f-%.0f m):\n", rules.max_block_weight,
                        rules.min_ring_length, rules.max_ring_length);
        } else {
            std::vector<double> xs;
            for (double f : manual_butt_frames) xs.push_back(st.frames.x(f));
            div = evaluate_division(wm, zones, p, st, xs, rules);
            std::printf("Manual division check (%zu butts):\n", manual_butt_frames.size());
        }
        if (!div.butts.empty()) {
            std::printf("  butts at FR:");
            for (size_t i = 1; i + 1 < div.butts.size(); ++i) std::printf(" %.1f", st.frames.frame(div.butts[i]));
            const Block* heavy = nullptr;
            for (auto& b : div.blocks) if (!heavy || b.mass.w > heavy->mass.w) heavy = &b;
            std::printf("\n  %zu rings, %zu blocks; heaviest %s %.0f t\n", div.butts.size() - 1, div.blocks.size(),
                        heavy ? heavy->name.c_str() : "-", heavy ? heavy->mass.w : 0.0);
            write_division_csv(div, outdir + "/blocks.csv");
            write_division_svg(div, mesh, p, st, zones, rules, outdir + "/block_division.svg");
            std::printf("  -> %s/blocks.csv, %s/block_division.svg\n", outdir.c_str(), outdir.c_str());
        }
        if (div.issues.empty()) std::printf("  all division rules met\n");
        for (auto& s : div.issues) std::printf("  RULE: %s\n", s.c_str());
    }

    if (!run_bench) return 0;

    // Throughput: 18 drafts x 25 heels x 5 trims = 2250 waterplanes against the full mesh.
    auto grid = condition_grid(p, 18, 25, 5);
    std::printf("\n== Benchmark: %zu waterplanes x %zu triangles ==\n", grid.size(), mesh.tris.size());
    auto serial = make_cpu_backend(1);
    serial->upload(mesh);
    double t1 = bench(*serial, grid, 3);
    std::printf("%-28s %8.3f s  (%.0f planes/s)\n", serial->name().c_str(), t1, grid.size() / t1);
    double tn = bench(*cpu, grid, 3);
    std::printf("%-28s %8.3f s  (%.0f planes/s, speed-up x%.2f)\n", cpu->name().c_str(), tn, grid.size() / tn,
                t1 / tn);
    if (auto gpu = make_cuda_backend()) {
        gpu->upload(mesh);
        double tg = bench(*gpu, grid, 3);
        std::vector<HydroAccum> a(grid.size()), b(grid.size());
        cpu->evaluate(grid, a);
        gpu->evaluate(grid, b);
        double maxrel = 0;
        for (size_t i = 0; i < a.size(); ++i)
            if (a[i].V > 1.0) maxrel = std::max(maxrel, std::fabs(a[i].V - b[i].V) / a[i].V);
        std::printf("%-28s %8.3f s  (%.0f planes/s, speed-up x%.2f, max rel. volume diff vs CPU %.1e)\n",
                    gpu->name().c_str(), tg, grid.size() / tg, t1 / tg, maxrel);
    } else {
        std::printf("cuda: not available (built without HULLCAD_ENABLE_CUDA or no GPU)\n");
    }
    return 0;
}
