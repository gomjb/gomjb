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
    YardProfile yard;
    std::vector<std::pair<std::string, std::string>> overrides;
    std::string sweep_key;
    std::vector<std::string> sweep_values;
    std::vector<double> manual_butt_frames;
    auto split = [](const std::string& s, char sep) {
        std::vector<std::string> out;
        std::stringstream ss(s);
        for (std::string t; std::getline(ss, t, sep);) out.push_back(t);
        return out;
    };
    try {
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            if (a == "--out" && i + 1 < argc) outdir = argv[++i];
            else if (a == "--threads" && i + 1 < argc) threads = std::atoi(argv[++i]);
            else if (a == "--no-bench") run_bench = false;
            else if (a == "--yard" && i + 1 < argc) yard = load_yard_profile(argv[++i], yard);
            else if (a == "--set" && i + 1 < argc) {
                std::string kv = argv[++i];
                size_t eq = kv.find('=');
                if (eq == std::string::npos) throw std::invalid_argument("--set expects section.key=value");
                set_yard_option(yard, kv.substr(0, eq), kv.substr(eq + 1));
            } else if (a == "--sweep" && i + 1 < argc) {
                std::string kv = argv[++i];
                size_t eq = kv.find('=');
                if (eq == std::string::npos) throw std::invalid_argument("--sweep expects section.key=v1,v2,...");
                sweep_key = kv.substr(0, eq);
                sweep_values = split(kv.substr(eq + 1), ',');
                YardProfile probe = yard;
                set_yard_option(probe, sweep_key, sweep_values.at(0));  // validate the key early
            } else if (a == "--write-yard" && i + 1 < argc) {
                write_yard_profile(yard, argv[++i]);
                std::printf("yard profile template written to %s\n", argv[i]);
                return 0;
            } else if (a == "--butts" && i + 1 < argc) {
                for (auto& t : split(argv[++i], ',')) manual_butt_frames.push_back(std::atof(t.c_str()));
            } else {
                std::printf(
                    "usage: hullcad_demo [--out DIR] [--threads N] [--no-bench]\n"
                    "                    [--yard FILE.ini] [--set section.key=value]... [--write-yard FILE.ini]\n"
                    "                    [--butts FR,FR,...] [--sweep section.key=v1,v2,...]\n");
                return 2;
            }
        }
    } catch (const std::exception& e) {
        std::printf("error: %s\n", e.what());
        return 2;
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
        ThreadPool pool(threads);
        auto tw = std::chrono::steady_clock::now();
        WeightModel wm = build_weight_model(mesh, p, st, default_cargo_zones(p, st, yard), default_end_zones(p, yard),
                                            0.25, pool);
        double t_wm = seconds_since(tw);
        Mass total = wm.ring_mass(0.0, p.Lpp);
        Mass cargo = wm.ring_mass(st.cargo_x0(), st.cargo_x1());
        std::printf("\n== Block division: %s ==\n", yard.name.c_str());
        std::printf("Weight model: %d slices x %d zones, %zu transverse members, built in %.3f s (%d threads)\n",
                    wm.nslice, wm.nzone, wm.transverse.size(), t_wm, pool.size());
        std::printf("Estimated hull steel (modelled members only): %.0f t total, %.0f t in cargo region, LCG %.1f m\n",
                    total.w, cargo.w, total.cog().x);

        Division div;
        auto td = std::chrono::steady_clock::now();
        if (manual_butt_frames.empty()) {
            div = auto_divide(wm, p, st, yard);
            std::printf("Automatic division (%.3f s):\n", seconds_since(td));
        } else {
            std::vector<double> xs;
            for (double f : manual_butt_frames) xs.push_back(st.frames.x(f));
            div = evaluate_division(wm, p, st, xs, yard);
            std::printf("Manual division check (%zu butts):\n", manual_butt_frames.size());
        }
        if (!div.butts.empty()) {
            std::printf("  butts at FR:");
            for (size_t i = 1; i + 1 < div.butts.size(); ++i) std::printf(" %.1f", st.frames.frame(div.butts[i]));
            int flat = 0;
            const Block* heavy = nullptr;
            for (auto& b : div.blocks) {
                flat += b.flat;
                if (!heavy || b.lift_weight > heavy->lift_weight) heavy = &b;
            }
            std::printf("\n  %zu rings, %zu blocks (%d flat, %zu curved); heaviest lift %s %.0f t\n",
                        div.butts.size() - 1, div.blocks.size(), flat, div.blocks.size() - flat,
                        heavy ? heavy->name.c_str() : "-", heavy ? heavy->lift_weight : 0.0);
            std::printf("  Facility utilisation (highest first):\n");
            for (auto& s : summarize_checks(div))
                std::printf("    %-26s max %5.1f %%  binding for %2d blocks  (worst %s)\n", s.name.c_str(),
                            100 * s.max_ratio, s.binding_blocks, s.worst_block.c_str());
            write_division_csv(div, outdir + "/blocks.csv");
            write_division_svg(div, mesh, p, st, wm, yard, outdir + "/block_division.svg");
            std::printf("  -> %s/blocks.csv, %s/block_division.svg\n", outdir.c_str(), outdir.c_str());
        }
        if (div.issues.empty()) std::printf("  all yard checks pass\n");
        for (auto& s : div.issues) std::printf("  RULE: %s\n", s.c_str());

        // What-if sweep over one yard option, reusing the weight model (zones depend only on
        // division.* keys, which therefore cannot be swept here).
        if (!sweep_key.empty()) {
            if (sweep_key.rfind("division.seam_offset", 0) == 0 || sweep_key.rfind("division.deck_zone_depth", 0) == 0 ||
                sweep_key.rfind("division.end_split_z", 0) == 0) {
                std::printf("sweep: %s changes the zones; run separate cases with --set instead\n", sweep_key.c_str());
            } else {
                std::printf("\n== What-if: %s ==\n", sweep_key.c_str());
                std::printf("  %-12s %6s %7s %10s  %s\n", "value", "rings", "blocks", "max util", "binding check (most blocks)");
                for (auto& v : sweep_values) {
                    YardProfile yv = yard;
                    set_yard_option(yv, sweep_key, v);
                    Division dv = auto_divide(wm, p, st, yv);
                    if (dv.butts.empty()) {
                        std::printf("  %-12s %6s %7s %10s  %s\n", v.c_str(), "-", "-", "-", "no feasible division");
                        continue;
                    }
                    auto sum = summarize_checks(dv);
                    const CheckSummary* most = nullptr;
                    for (auto& s : sum)
                        if (!most || s.binding_blocks > most->binding_blocks) most = &s;
                    std::printf("  %-12s %6zu %7zu %9.1f%%  %s (%d blocks)\n", v.c_str(), dv.butts.size() - 1,
                                dv.blocks.size(), sum.empty() ? 0.0 : 100 * sum.front().max_ratio,
                                most ? most->name.c_str() : "-", most ? most->binding_blocks : 0);
                }
            }
        }
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
