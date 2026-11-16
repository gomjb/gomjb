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

#include "hullcad/production_design/blocks.hpp"
#include "hullcad/naval/compute_backend.hpp"
#include "hullcad/cad/hull_param.hpp"
#include "hullcad/naval/hydrostatics.hpp"
#include "hullcad/cad/lines_plan.hpp"
#include "hullcad/cad/mesh.hpp"
#include "hullcad/cad/offsets_io.hpp"
#include "hullcad/fairing/fairing.hpp"
#include "hullcad/fairing/network.hpp"
#include "hullcad/planning/schedule.hpp"
#include "hullcad/production/daily_load.hpp"
#include "hullcad/production/utilities.hpp"
#include "hullcad/project/yard_profile.hpp"

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
    bool bilge_g1 = false, fair_surface = false, raw_surface = false;
    int network = -1;  // -1: default (on for imported tables), 0 off, 1 on
    std::string offsets_file, export_offsets_file;
    double offset_accuracy_mm = 1.0, offset_noise_mm = 0.0, station_error_mm = 0.0;
    int offset_points = 25;
    double fair_tolerance_mm = 3.0, network_tolerance_mm = 5.0;
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
            else if (a == "--bilge" && i + 1 < argc) {
                std::string b = argv[++i];
                if (b != "g1" && b != "g2") throw std::invalid_argument("--bilge expects g1 or g2");
                bilge_g1 = b == "g1";
            } else if (a == "--fair-surface") fair_surface = true;
            else if (a == "--network") network = 1;
            else if (a == "--no-network") network = 0;
            else if (a == "--network-tolerance" && i + 1 < argc) network_tolerance_mm = std::atof(argv[++i]);
            else if (a == "--fair-tolerance" && i + 1 < argc) fair_tolerance_mm = std::atof(argv[++i]);
            else if (a == "--offsets" && i + 1 < argc) offsets_file = argv[++i];
            else if (a == "--offset-accuracy" && i + 1 < argc) offset_accuracy_mm = std::atof(argv[++i]);
            else if (a == "--raw-surface") raw_surface = true;
            else if (a == "--export-offsets" && i + 1 < argc) export_offsets_file = argv[++i];
            else if (a == "--offset-points" && i + 1 < argc) offset_points = std::atoi(argv[++i]);
            else if (a == "--offset-noise" && i + 1 < argc) offset_noise_mm = std::atof(argv[++i]);
            else if (a == "--station-error" && i + 1 < argc) station_error_mm = std::atof(argv[++i]);
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
                    "                    [--butts FR,FR,...] [--sweep section.key=v1,v2,...]\n"
                    "                    [--bilge g2|g1] [--fair-surface] [--fair-tolerance MM] [--network|--no-network] [--network-tolerance MM]\n"
                    "                    [--offsets FILE.csv [--offset-accuracy MM] [--raw-surface]]\n"
                    "                    [--export-offsets FILE.csv [--offset-points N] [--offset-noise MM] [--station-error MM]]\n");
                return 2;
            }
        }
    } catch (const std::exception& e) {
        std::printf("error: %s\n", e.what());
        return 2;
    }
    std::filesystem::create_directories(outdir);

    TankerParams p;
    if (bilge_g1) p.n_mid = 2.0;  // circular bilge (default: curvature-continuous)

    if (!export_offsets_file.empty()) {
        // Offset table of the parametric hull, rounded to the millimetre like a printed table.
        write_offsets_csv(parametric_offset_table(p, offset_points, offset_noise_mm / 1000.0, 0.001, 1, station_error_mm / 1000.0),
                          export_offsets_file);
        std::printf("offset table (%d stations x %d points, noise +-%.1f mm, station error +-%.1f mm) written to %s\n", p.stations,
                    offset_points, offset_noise_mm, station_error_mm, export_offsets_file.c_str());
        return 0;
    }
    OffsetTable table;
    const bool imported = !offsets_file.empty();
    if (imported) {
        try {
            table = read_offsets_csv(offsets_file);
        } catch (const std::exception& e) {
            std::printf("error: %s\n", e.what());
            return 2;
        }
        p = params_for_table(table, p);
        const OffsetTableInfo info = describe(table);
        std::printf("Offset table %s: %d stations, %d-%d points, Lpp %.2f m, B %.2f m, D %.2f m (x shifted by %.3f m)\n",
                    offsets_file.c_str(), info.stations, info.min_points, info.max_points, p.Lpp, p.B, p.D, table.x_shift);
    }
    ShipRef ref{p.Lpp, p.B, 0.5 * p.Lpp, 1.025};

    auto t0 = std::chrono::steady_clock::now();
    // Section fairing (always analysed). Parametric hull: the surface uses the design sections
    // unless --fair-surface. Imported table: the surface uses the faired sections unless --raw-surface.
    FairingOptions fo;
    fo.tolerance = fair_tolerance_mm / 1000.0;
    fo.junction_continuity = p.n_mid > 2.5 ? 2 : 1;
    if (imported) fo.offset_accuracy = offset_accuracy_mm / 1000.0;
    HullFairing hf = imported ? fair_offset_table(table, fo, p.points_per_section) : fair_hull_sections(p, fo);
    if (imported) fair_surface = !raw_surface;
    if (network == 1) fair_surface = true;
    const bool network_fair = fair_surface && (network == 1 || (network == -1 && imported));
    NetworkOptions no = default_network(p);
    no.tolerance = network_tolerance_mm / 1000.0;
    no.offset_accuracy = fo.offset_accuracy;
    no.fair_girth_lines = network_fair;
    const int stripes_before = surface_curvature_map(fair_surface ? build_faired_hull_surface(hf, p) : build_offset_surface(hf))
                                   .long_sign_changes();
    const NetworkResult net = fair_network(hf, no);
    BSplineSurface S = fair_surface ? build_faired_hull_surface(hf, p) : build_offset_surface(hf);
    TriMesh mesh = tessellate_hull(S, 240, 80);
    std::printf("Hull surface: bicubic B-spline, %d x %d control points (from %zu stations x %d points, %s sections)\n",
                S.nu, S.nv, hf.sections.size(), fair_surface ? fo.surface_points : p.points_per_section,
                fair_surface ? "faired" : "unfaired");
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

    // Offsets of the sections the surface was built from.
    {
        OffsetTable used;
        for (auto& sc : hf.sections) used.stations.push_back(fair_surface ? sc.faired : sc.offsets);
        write_offsets_csv(used, outdir + "/offsets.csv");
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

    {
        const FairingSummary fs = summarize(hf);
        std::printf("\n== Section fairing (%s, %s joints, tolerance %.1f mm, surface from %s sections) ==\n",
                    imported ? "imported offset table" : p.n_mid > 2.5 ? "curvature-continuous bilge" : "circular bilge", fo.junction_continuity >= 2 ? "G2" : "G1",
                    fair_tolerance_mm, fair_surface ? "faired" : "design");
        std::printf("%d stations, %d faired, %d not applicable; max deviation %.1f mm; inflections %d -> %d\n", fs.stations,
                    fs.stations - fs.not_applicable, fs.not_applicable, fs.max_deviation * 1000, fs.inflections_before,
                    fs.inflections_faired);
        std::printf("  %8s %9s %12s %14s  %s\n", "x", "dev mm", "inflections", "energy", "status");
        for (size_t i = 0; i < hf.sections.size(); i += 5) {
            const auto& sc = hf.sections[i];
            std::printf("  %8.1f %9.1f %5d -> %-4d %6.3f -> %-6.3f  %s\n", sc.x, sc.max_deviation * 1000, sc.q_before.inflections,
                        sc.q_composite.inflections, sc.q_before.energy, sc.q_composite.energy, sc.ok ? "faired" : "not applicable");
        }
    }

    {
        const int stripes = surface_curvature_map(S).long_sign_changes();
        if (net.girth_lines > 0)
            std::printf("\n== Curve network (girth lines faired, tolerance %.1f mm) ==\n", network_tolerance_mm);
        else
            std::printf("\n== Curve network (inspection only) ==\n");
        if (net.girth_lines > 0)
            std::printf("%d girth lines (%d with a straight middle body), largest section point move %.1f mm\n", net.girth_lines,
                        net.straight_lines, net.max_move * 1000);
        std::printf("surface longitudinal curvature sign changes: %d%s\n", net.girth_lines > 0 ? stripes_before : stripes,
                    net.girth_lines > 0 ? (" -> " + std::to_string(stripes)).c_str() : "");
        std::printf("  %-10s %6s %24s %28s\n", "curve", "points", "inflections raw/sec/net", "curvature variation raw/sec/net");
        for (auto& c : net.curves)
            std::printf("  %-10s %6zu %8d %6d %6d %12.3f %8.3f %8.3f\n", c.name().c_str(), c.sectioned.size(), c.q_raw.inflections,
                        c.q_sectioned.inflections, c.q_faired.inflections, c.q_raw.variation, c.q_sectioned.variation,
                        c.q_faired.variation);
    }

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

        // Utilities: work quantities of every block x the yard's unit consumption (reference values).
        if (!div.blocks.empty()) {
            TankVolumes tv = tank_volumes(mesh, p, st);
            UtilityReport ur = estimate_utilities(div, tv, yard);
            const auto& Q = ur.quantities;
            std::printf("\n== Utilities (unit rates are reference values; replace with yard data) ==\n");
            std::printf("Quantities: plate %.0f m2, coated %.0f m2, cutting %.0f m, fillet %.0f m, shop butt %.0f m, "
                        "erection %.0f m, line heating %.0f m\n",
                        Q.plate_area, Q.paint_area, Q.cut_length, Q.fillet_length, Q.shop_butt_length,
                        Q.erection_length, Q.heating_length);
            std::printf("Weld metal %.1f t (FCAW %.1f t, SAW %.1f t); tanks: cargo %.0f m3, ballast %.0f m3\n",
                        ur.weld_metal_kg / 1000, ur.fcaw_kg / 1000, ur.saw_kg / 1000, tv.cargo_m3, tv.ballast_m3);
            std::printf("  %-12s", "stage");
            for (auto& it : utility_items()) std::printf(" %12s", it.key);
            std::printf("\n");
            auto row = [&](const char* name, const UtilityUse& u) {
                std::printf("  %-12s", name);
                for (auto& it : utility_items()) std::printf(" %12.0f", u.*(it.member));
                std::printf("\n");
            };
            for (int s = 0; s < kStageCount; ++s) row(stage_key(static_cast<Stage>(s)), ur.by_stage[s]);
            row("TOTAL", ur.total);
            std::printf("  per tonne of steel: %.0f kWh, %.1f Nm3 CO2, %.0f Nm3 air, %.1f L paint\n",
                        ur.total.electricity_kwh / ur.steel_t, ur.total.co2_nm3 / ur.steel_t,
                        ur.total.air_nm3 / ur.steel_t, ur.total.paint_l / ur.steel_t);
            write_utilities_csv(ur, outdir + "/utilities.csv");
            std::printf("  -> %s/utilities.csv\n", outdir.c_str());

            // Schedule: erection sequence, stages planned backwards, daily profile and peaks.
            ProductionSchedule ps = schedule_blocks(div, yard);               // planning
            DailyUtilityProfile sr = daily_utilities(ps, ur, yard.capacity);  // production
            std::printf("\n== Schedule and daily utility peaks ==\n");
            std::printf("First cutting %s, erection %s .. %s, tank tests %s .. %s (%d working days)\n",
                        sr.calendar.date(sr.first_wd).c_str(), ps.calendar.date(0).c_str(),
                        ps.calendar.date(ps.last_erection_wd).c_str(),
                        ps.calendar.date(ps.testing.start).c_str(), ps.calendar.date(ps.testing.end - 1).c_str(), sr.days());
            std::printf("  %-16s %14s %11s %12s %12s %10s %8s  %s\n", "utility", "peak/day", "date", "mean/day",
                        "peak rate", "capacity", "use", "peak day: dominant stage, largest item");
            for (auto& pk : sr.peaks) {
                std::string top = "-";
                if (pk.dominant_stage >= 0) {
                    const char* name = pk.dominant_stage < kStageCount ? stage_key(static_cast<Stage>(pk.dominant_stage)) : "base load";
                    top = std::string(name) + " " + std::to_string(static_cast<int>(std::lround(100 * pk.stage_share[pk.dominant_stage]))) + "%";
                    if (!pk.contributors.empty()) top += ", largest item: " + pk.contributors.front().first;
                }
                std::printf("  %-16s %14.0f %11s %12.0f %8.1f %-7s %6.0f %7s  %s\n", pk.key.c_str(), pk.daily_peak,
                            sr.calendar.date(pk.peak_wd).c_str(), pk.daily_mean, pk.peak_rate, pk.rate_unit.c_str(),
                            pk.capacity, pk.capacity > 0 ? (std::to_string(static_cast<int>(std::lround(100 * pk.utilization))) + "%").c_str() : "-",
                            top.c_str());
            }
            for (auto& pk : sr.peaks)
                if (pk.capacity > 0 && pk.days_over > 0)
                    std::printf("  CAPACITY: %s exceeds %.0f %s on %d day(s), peak %.0f %%\n", pk.key.c_str(), pk.capacity,
                                pk.rate_unit.c_str(), pk.days_over, 100 * pk.utilization);
            write_block_schedule_csv(ps, outdir + "/block_schedule.csv");
            write_daily_utilities_csv(sr, outdir + "/daily_utilities.csv");
            std::printf("  -> %s/block_schedule.csv, %s/daily_utilities.csv\n", outdir.c_str(), outdir.c_str());
        }

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
