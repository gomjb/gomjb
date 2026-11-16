// Self-contained test runner (no external framework).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <vector>

#include "hullcad/production_design/blocks.hpp"
#include "hullcad/cad/bspline.hpp"
#include "hullcad/naval/compute_backend.hpp"
#include "hullcad/cad/hull_param.hpp"
#include "hullcad/naval/hydrostatics.hpp"
#include "hullcad/cad/lines_plan.hpp"
#include "hullcad/cad/mesh.hpp"
#include "hullcad/cad/offsets_io.hpp"
#include "hullcad/fairing/fairing.hpp"
#include "hullcad/planning/schedule.hpp"
#include "hullcad/production/daily_load.hpp"
#include "hullcad/production/utilities.hpp"
#include "hullcad/project/yard_profile.hpp"

using namespace hullcad;

static int g_fail = 0, g_pass = 0;
#define CHECK_NEAR(a, b, tol)                                                                   \
    do {                                                                                        \
        double _a = (a), _b = (b);                                                              \
        if (std::fabs(_a - _b) > (tol)) {                                                       \
            std::printf("FAIL %s:%d  %s = %.10g, expected %.10g (tol %g)\n", __FILE__, __LINE__, \
                        #a, _a, _b, (double)(tol));                                             \
            ++g_fail;                                                                           \
        } else {                                                                                \
            ++g_pass;                                                                           \
        }                                                                                       \
    } while (0)
#define CHECK(c)                                                              \
    do {                                                                      \
        if (!(c)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++g_fail; } \
        else ++g_pass;                                                        \
    } while (0)

static void test_bspline() {
    // Interpolation passes through the data; curvature of a sampled circle ~ 1/R.
    const double R = 10.0;
    std::vector<Vec3> pts;
    for (int i = 0; i <= 30; ++i) {
        double t = 3.14159265358979 * i / 30.0;
        pts.push_back({R * std::cos(t), R * std::sin(t), 0.0});
    }
    auto prm = chord_length_params(pts);
    auto c = interpolate(pts, 3, prm, averaged_knots(prm, 3));
    for (size_t i = 0; i < pts.size(); ++i) CHECK_NEAR(norm(c.eval(prm[i]) - pts[i]), 0.0, 1e-9);
    CHECK_NEAR(c.curvature(0.5), 1.0 / R, 1e-3 / R);
    CHECK_NEAR(norm(c.eval(0.37)), R, 1e-4);

    // First derivative matches a central finite difference.
    Vec3 d[3];
    c.derivs(0.4, 2, d);
    double h = 1e-6;
    Vec3 fd = (c.eval(0.4 + h) - c.eval(0.4 - h)) * (0.5 / h);
    CHECK_NEAR(norm(d[1] - fd), 0.0, 1e-5 * norm(d[1]));

    // Skinned surface interpolates its sections at the data parameters (corners exactly).
    std::vector<std::vector<Vec3>> secs;
    for (int k = 0; k < 6; ++k) {
        std::vector<Vec3> s;
        for (int j = 0; j < 7; ++j) s.push_back({10.0 * k, 1.0 * j + 0.1 * k * k, std::sin(0.5 * j) + 0.2 * k});
        secs.push_back(s);
    }
    auto S = skin(secs, 3, 3);
    CHECK_NEAR(norm(S.eval(0.0, 0.0) - secs[0][0]), 0.0, 1e-9);
    CHECK_NEAR(norm(S.eval(1.0, 1.0) - secs[5][6]), 0.0, 1e-9);
}

static void test_box_upright() {
    const double L = 100, B = 20, D = 10, T = 4;
    TriMesh box = make_box(L, B, D);
    CHECK(is_closed_and_consistent(box));
    CHECK_NEAR(signed_volume(box), L * B * D, 1e-6);

    auto be = make_cpu_backend(4);
    be->upload(box);
    ShipRef ref{L, B, L / 2, 1.025};
    auto tab = hydrostatic_table(*be, ref, {T});
    const HydroResult& r = tab[0];
    CHECK_NEAR(r.volume, L * B * T, 1e-6);
    CHECK_NEAR(r.B.x, L / 2, 1e-9);
    CHECK_NEAR(r.B.y, 0.0, 1e-9);
    CHECK_NEAR(r.B.z, T / 2, 1e-9);
    CHECK_NEAR(r.Awp, L * B, 1e-6);
    CHECK_NEAR(r.F.x, L / 2, 1e-9);
    CHECK_NEAR(r.BM_T, B * B / (12 * T), 1e-9);
    CHECK_NEAR(r.BM_L, L * L / (12 * T), 1e-6);
    CHECK_NEAR(r.Cb, 1.0, 1e-12);
}

static void test_box_heel_and_trim() {
    const double L = 100, B = 20, D = 10, T = 4;
    TriMesh box = make_box(L, B, D);
    auto be = make_cpu_backend(2);
    be->upload(box);
    ShipRef ref{L, B, L / 2, 1.025};

    // A heeled or trimmed plane through the centre of a symmetric box keeps the volume.
    for (double heel : {5.0, 15.0}) {
        WaterPlane wp = make_plane(T, heel, 0.0, L / 2);
        HydroAccum a;
        be->evaluate(std::span<const WaterPlane>(&wp, 1), std::span<HydroAccum>(&a, 1));
        CHECK_NEAR(a.V, L * B * T, 1e-6);
    }
    {
        WaterPlane wp = make_plane(T, 0.0, 1.0, L / 2);
        HydroAccum a;
        be->evaluate(std::span<const WaterPlane>(&wp, 1), std::span<HydroAccum>(&a, 1));
        CHECK_NEAR(a.V, L * B * T, 1e-6);
    }

    // Wall-sided formula: GZ = sin(phi) (GM + BM/2 tan^2 phi), valid before deck-edge
    // immersion / bilge emergence (tan(phi) < 0.4 here).
    const double KG = 5.0;
    const double BM = B * B / (12 * T), GM = T / 2 + BM - KG;
    std::vector<double> heels{0, 5, 10, 15, 20};
    auto gz = gz_curve(*be, ref, L * B * T, Vec3{L / 2, 0, KG}, heels);
    for (size_t i = 0; i < heels.size(); ++i) {
        double ph = heels[i] * 3.14159265358979 / 180.0;
        double expect = std::sin(ph) * (GM + 0.5 * BM * std::tan(ph) * std::tan(ph));
        CHECK_NEAR(gz[i].GZ, expect, 1e-7);
        CHECK_NEAR(gz[i].draft, T, 1e-6);
    }
}

static void test_tanker_mesh_and_determinism() {
    TankerParams p;
    auto S = build_hull_surface(p);
    TriMesh m = tessellate_hull(S, 120, 40);
    CHECK(is_closed_and_consistent(m));
    CHECK(signed_volume(m) > 0);

    // Mesh refinement changes the design-draft volume by well under 0.1 %.
    TriMesh fine = tessellate_hull(S, 240, 80);
    ShipRef ref{p.Lpp, p.B, p.Lpp / 2, 1.025};
    auto b1 = make_cpu_backend(4);
    b1->upload(m);
    double v_coarse = hydrostatic_table(*b1, ref, {p.T})[0].volume;
    b1->upload(fine);
    double v_fine = hydrostatic_table(*b1, ref, {p.T})[0].volume;
    CHECK(std::fabs(v_coarse - v_fine) / v_fine < 1e-3);

    // Bit-identical results for 1 vs 4 threads, and for "few planes" vs "many planes" paths.
    std::vector<WaterPlane> planes;
    for (int i = 0; i < 12; ++i) planes.push_back(make_plane(4.0 + i, 3.0 * i, 0.2 * (i % 3), p.Lpp / 2));
    auto one = make_cpu_backend(1), four = make_cpu_backend(4);
    one->upload(fine);
    four->upload(fine);
    std::vector<HydroAccum> r1(planes.size()), r4(planes.size()), r4single(planes.size());
    one->evaluate(planes, r1);
    four->evaluate(planes, r4);
    for (size_t i = 0; i < planes.size(); ++i)
        four->evaluate(std::span<const WaterPlane>(&planes[i], 1), std::span<HydroAccum>(&r4single[i], 1));
    CHECK(std::memcmp(r1.data(), r4.data(), r1.size() * sizeof(HydroAccum)) == 0);
    CHECK(std::memcmp(r1.data(), r4single.data(), r1.size() * sizeof(HydroAccum)) == 0);
}

static void test_frame_table() {
    FrameTable ft({{0, 0.80}, {50, 0.95}, {240, 0.80}});
    CHECK_NEAR(ft.x(50), 40.0, 1e-12);
    CHECK_NEAR(ft.x(240), 40.0 + 190 * 0.95, 1e-9);
    CHECK_NEAR(ft.x(-2), -1.6, 1e-12);
    for (double f : {-3.0, 0.0, 12.5, 49.99, 50.0, 133.3, 240.0, 255.5})
        CHECK_NEAR(ft.frame(ft.x(f)), f, 1e-9);
    CHECK_NEAR(ft.spacing_at(100), 0.95, 0.0);
}

// Box hull with shell, deck and one transverse bulkhead: block weights have a closed form,
// and the zones of each scheme must partition the section exactly (no gaps, no double counting).
static void test_box_block_weights() {
    TankerParams p;
    p.Lpp = 60; p.B = 20; p.D = 10;
    TriMesh box = make_box(p.Lpp, p.B, p.D);
    TankerStructure st;
    st.frames = FrameTable({{0, 1.0}});
    st.cargo_aft_frame = 1000; st.cargo_fwd_frame = 1100;  // no double hull in this test
    st.bulkhead_frames = {31};
    YardProfile y;
    auto cz = default_cargo_zones(p, st, y);
    auto ez = default_end_zones(p, y);
    ThreadPool pool(4);
    WeightModel wm = build_weight_model(box, p, st, cz, ez, 0.25, pool);
    CHECK(wm.nzone == 12 && wm.n_cargo_zones == 8);

    auto mpa = [](const PlateSpec& s) { return s.t_mm * 1e-3 * 7.85 * (1 + s.stiff_factor); };
    const double per_m = p.B * mpa(st.bottom_shell) + 2 * p.D * mpa(st.side_shell) + p.B * mpa(st.deck);
    const double bhd = p.B * p.D * mpa(st.trans_bulkhead);
    CHECK_NEAR(wm.ring_mass(0, p.Lpp).w, p.Lpp * per_m + bhd, 1e-6);
    CHECK_NEAR(wm.ring_mass(10, 20).w, 10 * per_m, 1e-6);           // no bulkhead in range
    CHECK_NEAR(wm.ring_mass(25, 35).w, 10 * per_m + bhd, 1e-6);     // bulkhead at x = 31
    CHECK_NEAR(wm.ring_mass(10.3, 17.8).w, 7.5 * per_m, 1e-6);      // fractional slices
    CHECK_NEAR(wm.ring_mass(0, p.Lpp).cog().y, 0.0, 1e-9);
    // The end-body scheme partitions the same section.
    double end_total = 0;
    for (int z = wm.n_cargo_zones; z < wm.nzone; ++z) end_total += wm.zone_mass(z, 0, p.Lpp).w;
    CHECK_NEAR(end_total, p.Lpp * per_m + bhd, 1e-6);
    // Bottom shell of DB-P: y from the CL seam (0.3) to the side seam (min(B/2-2, B/2-R) - 0.3).
    const double a = std::min(p.B / 2 - 2.0, p.B / 2 - p.bilge_radius) - 0.3;
    CHECK(wm.zones[1].name == "DB-P");
    CHECK_NEAR(wm.zone_mass(1, 10, 20).w, (a - 0.3) * mpa(st.bottom_shell) * 10, 1e-6);

    // Box blocks are flat where the plating is planar; the DB block has the box's extents.
    Block db = evaluate_block(wm, 1, 10, 20, y);
    CHECK(db.flat);
    CHECK_NEAR(db.L, 10, 1e-12);
    CHECK_NEAR(db.B, a - 0.3, 1e-9);
    CHECK_NEAR(db.lift_weight, db.mass.w * 1.03 * 1.05 + 8.0, 1e-9);

    // Extra longitudinal seams keep the partition exact.
    YardProfile ys = y;
    ys.division.long_seams_y = {4.0};
    WeightModel ws = build_weight_model(box, p, st, default_cargo_zones(p, st, ys), default_end_zones(p, ys), 0.25, pool);
    CHECK(ws.n_cargo_zones == 14);  // DB, BHD, DECK x 2 bands x P/S + 2 sides
    CHECK(ws.zones[0].name == "DB-1-S" && ws.zones[3].name == "DB-2-P");
    CHECK_NEAR(ws.ring_mass(0, p.Lpp).w, p.Lpp * per_m + bhd, 1e-6);

    // Same model on one thread is bit-identical.
    ThreadPool one(1);
    WeightModel w1 = build_weight_model(box, p, st, cz, ez, 0.25, one);
    CHECK(std::memcmp(w1.prefix.data(), wm.prefix.data(), wm.prefix.size() * sizeof(Mass)) == 0);
}

// Quantity takeoff and utility estimate on a box hull, where every quantity has a closed form.
static void test_box_takeoff_and_utilities() {
    TankerParams p;
    p.Lpp = 60; p.B = 20; p.D = 10;
    TriMesh box = make_box(p.Lpp, p.B, p.D);
    TankerStructure st;
    st.frames = FrameTable({{0, 1.0}});
    st.cargo_aft_frame = 1000; st.cargo_fwd_frame = 1100;  // shell and deck only
    YardProfile y;
    ThreadPool pool(4);
    WeightModel wm = build_weight_model(box, p, st, default_cargo_zones(p, st, y), default_end_zones(p, y), 0.25, pool);

    // DB-P block, x in [10, 20): bottom shell from the CL seam (0.3) to the side seam (7.3).
    const double girth = 7.0, L = 10.0, sp = st.stiffener_spacing;
    QuantityTakeoff q = takeoff_block(wm, 1, 10, 20, true, st, y);
    CHECK_NEAR(q.plate_area, girth * L, 1e-9);
    CHECK_NEAR(q.stiffener_length, girth / sp * L, 1e-9);
    CHECK_NEAR(q.fillet_length, 2 * girth / sp * L, 1e-9);           // no plate joints in this zone
    CHECK_NEAR(q.shop_butt_length, 1 * L, 1e-9);                     // ceil(7 / 4.5) - 1 = 1 seam, no butt
    CHECK_NEAR(q.erection_length, L + girth, 1e-9);                  // seam to SIDE-P + aft butt
    CHECK_NEAR(q.paint_area, 2 * girth * L + girth / sp * L * y.takeoff.stiffener_paint_girth, 1e-9);
    CHECK_NEAR(q.cut_length, girth * L * 2 * (21.0 + 4.5) / (21.0 * 4.5), 1e-9);
    CHECK_NEAR(q.mean_thickness, st.bottom_shell.t_mm, 1e-9);
    CHECK_NEAR(q.heating_length, 0.0, 0.0);
    CHECK(takeoff_block(wm, 1, 10, 20, false, st, y).heating_length > 0);

    // SIDE-P holds the deck-edge joint: two fillet sides per metre on top of the stiffeners.
    QuantityTakeoff side = takeoff_block(wm, 3, 10, 20, false, st, y);
    const double side_girth = wm.slices[(15 * 4) * wm.nzone + 3].plate_len;
    CHECK_NEAR(side.fillet_length, 2 * (side_girth / sp + 1) * L, 1e-6);

    // Utilities of one flat block equal quantity x rate, stage by stage.
    Division d;
    Block b = evaluate_block(wm, 1, 10, 20, y);
    b.name = "T-DB-P";
    b.q = q;
    d.blocks.push_back(b);
    TankVolumes tv{1000.0, 500.0};
    UtilityReport r = estimate_utilities(d, tv, y);
    const double fillet_kg = q.fillet_length * y.util_welding.fillet_deposit;
    const double saw_kg = q.shop_butt_length * y.util_welding.shop_butt_deposit;  // flat block -> SAW
    const auto& W = r.by_stage[static_cast<int>(Stage::Welding)];
    CHECK_NEAR(W.electricity_kwh, fillet_kg * y.util_welding.fcaw_kwh_per_kg + saw_kg * y.util_welding.saw_kwh_per_kg, 1e-9);
    CHECK_NEAR(W.co2_nm3, fillet_kg * y.util_welding.fcaw_co2_nm3_per_kg, 1e-9);
    const auto& P = r.by_stage[static_cast<int>(Stage::Painting)];
    CHECK_NEAR(P.paint_l, q.paint_area * y.util_painting.coats * y.util_painting.paint_l_per_m2_coat, 1e-9);
    const auto& C = r.by_stage[static_cast<int>(Stage::Cutting)];
    CHECK_NEAR(C.electricity_kwh, q.cut_length * y.util_cutting.plasma_kwh_per_m, 1e-9);
    const auto& H = r.by_stage[static_cast<int>(Stage::Handling)];
    CHECK_NEAR(H.electricity_kwh, 4 * b.lift_weight * 1000 * 9.81 * 25 / 3.6e6 / 0.5, 1e-9);
    const auto& T = r.by_stage[static_cast<int>(Stage::Testing)];
    CHECK_NEAR(T.water_m3, 1500 * 0.5, 1e-9);
    CHECK_NEAR(T.air_nm3, 1500 * 0.5 * (0.2 / 1.01325) * 2.0, 1e-9);
    double sum = 0;
    for (int s = 0; s < kStageCount; ++s) sum += r.by_stage[s].electricity_kwh;
    CHECK_NEAR(r.total.electricity_kwh, sum, 1e-9);
    CHECK_NEAR(r.weld_metal_kg, fillet_kg + saw_kg + q.erection_length * y.util_welding.erection_butt_deposit, 1e-9);

    // Doubling a rate doubles exactly the consumption it drives.
    YardProfile y2 = y;
    y2.util_painting.blast_air_nm3_per_m2 *= 2;
    UtilityReport r2 = estimate_utilities(d, tv, y2);
    CHECK_NEAR(r2.by_stage[static_cast<int>(Stage::Painting)].air_nm3 - P.air_nm3, q.paint_area * y.util_painting.blast_air_nm3_per_m2, 1e-6);
    CHECK_NEAR(r2.by_stage[static_cast<int>(Stage::Welding)].air_nm3, W.air_nm3, 0.0);

    // Tank volumes: cargo region x in [10, 50), inner hull 2 m inboard, double bottom 2 m.
    TankerStructure st2 = st;
    st2.cargo_aft_frame = 10; st2.cargo_fwd_frame = 50;
    TankVolumes v = tank_volumes(box, p, st2);
    CHECK_NEAR(v.cargo_m3, (p.B - 4) * (p.D - 2) * 40, 1e-6);
    CHECK_NEAR(v.ballast_m3, (p.B * p.D - (p.B - 4) * (p.D - 2)) * 40, 1e-6);
}

// Deterministic +-amp noise added to the interior points (measured / entered offsets).
static std::vector<Vec3> with_noise(std::vector<Vec3> pts, double amp, unsigned seed) {
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        seed = seed * 1664525u + 1013904223u;
        const double a = ((seed >> 8) / 16777216.0 - 0.5) * 2;
        seed = seed * 1664525u + 1013904223u;
        const double b = ((seed >> 8) / 16777216.0 - 0.5) * 2;
        pts[i].y += a * amp;
        pts[i].z += b * amp;
    }
    return pts;
}

static void test_fairing() {
    // 1. Noisy quarter circle: fairing removes the bumps, stays within tolerance, keeps 1/R.
    const double R = 3.0;
    std::vector<Vec3> arc;
    for (int i = 0; i <= 30; ++i) {
        const double t = 1.5707963267948966 * i / 30;
        arc.push_back({0, R * std::sin(t), R - R * std::cos(t)});
    }
    auto noisy = with_noise(arc, 0.002, 7);
    FairingOptions o;
    o.tolerance = 0.004;
    o.offset_accuracy = 0.002;
    FairedCurve fc = fair_curve(noisy, o);
    CHECK(fc.max_deviation <= o.tolerance + 1e-9);
    CHECK(curve_quality(curvature_samples(interpolate(noisy, 3), 600)).inflections > 5);
    CHECK(curve_quality(curvature_samples(fc, 600)).inflections == 0);
    auto ks = curvature_samples(fc, 101);
    for (int i = 20; i <= 80; ++i) CHECK_NEAR(ks[i].kappa, 1.0 / R, 0.05 / R);

    // 2. A 49-point midship offset table (flat + circular bilge + flat side) with +-1 mm noise:
    //    straight runs found on their least-squares lines, bilge faired, no inflections.
    TankerParams p;
    const double x = 119.5;
    auto table = section_points(section_at(p, x), x, p.D, 49);
    auto table_noisy = with_noise(table, 0.001, 99);
    FairingOptions so;
    so.junction_continuity = 1;
    so.offset_accuracy = 0.001;
    so.start_tangent = {0, 1, 0};
    so.end_tangent = {0, 0, 1};
    FairedCurve sec = fair_curve(table_noisy, so);
    CHECK(sec.max_deviation <= so.tolerance + 1e-9);
    CHECK(curve_quality(curvature_samples(interpolate(table_noisy, 3), 1200)).inflections > 10);
    CHECK(curve_quality(curvature_samples(sec, 1200)).inflections == 0);
    CHECK(std::fabs(sec.lead_a.z) < 0.001 && std::fabs(sec.lead_b.z) < 0.001);    // flat of bottom
    CHECK(std::fabs(sec.tail_a.y - 22.0) < 0.001 && std::fabs(sec.tail_b.y - 22.0) < 0.001);  // flat of side

    // 3. Exact offsets, G1: the flat runs end at the bilge tangent points (y = B/2 - R, z = R).
    FairingOptions exact = so;
    exact.offset_accuracy = 0;
    auto dense = section_points(section_at(p, x), x, p.D, 481);
    FairedCurve g1 = fair_curve(dense, exact);
    CHECK(g1.max_deviation <= so.tolerance + 1e-9);
    CHECK(std::fabs(g1.lead_b.y - (22.0 - 2.4)) < 0.12);
    CHECK(std::fabs(g1.tail_a.z - 2.4) < 0.12);
    CHECK_NEAR(g1.lead_b.z, 0.0, 1e-4);  // flat within 0.1 mm

    // 4. Curvature-continuous bilge (n_mid = 4), G2: curvature rises from zero, no inflections.
    TankerParams p4;
    p4.n_mid = 4.0;
    FairingOptions g2o = so;
    g2o.junction_continuity = 2;
    g2o.offset_accuracy = 0;  // exact offsets
    FairedCurve g2 = fair_curve(section_points(section_at(p4, x), x, p4.D, 481), g2o);
    CHECK(g2.max_deviation <= g2o.tolerance + 1e-9);
    auto k2 = curvature_samples(g2.curve, 400);
    CHECK(std::fabs(k2.front().kappa) < 0.01 && std::fabs(k2.back().kappa) < 0.01);
    CHECK(curve_quality(curvature_samples(g2, 1200)).inflections == 0);

    // 5. Without a straight run the start tangent is honoured (V section aft: horizontal at keel).
    const double xa = 20.0;
    FairedCurve v = fair_curve(section_points(section_at(p, xa), xa, p.D, 481), exact);
    Vec3 dv[3];
    v.curve.derivs(v.curve.umin(), 1, dv);
    if (norm(v.lead_b - v.lead_a) < 1e-9) CHECK_NEAR(std::fabs(normalize(dv[1]).y), 1.0, 1e-9);

    // 6. The n_mid = 4 bilge removes the same section area as the circular one.
    auto area = [&](const TankerParams& q) {
        auto pts = section_points(section_at(q, x), x, q.D, 4001);
        pts.push_back({x, 0.0, q.D});
        double a = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            const Vec3& u = pts[i];
            const Vec3& w = pts[(i + 1) % pts.size()];
            a += u.y * w.z - w.y * u.z;
        }
        return std::fabs(a) / 2;
    };
    CHECK_NEAR(bilge_corner_size(p), p.bilge_radius, 0.0);
    CHECK_NEAR(area(p4), area(p), 1e-3);

    // 7. Hull: the curvature-continuous bilge removes the flat-bottom ripple of the surface.
    auto ripple = [&](const TriMesh& m, const TankerParams& q) {
        const double fob = q.B / 2 - bilge_corner_size(q);
        double r = 0;
        for (double xx : {100.0, 140.0}) {
            double lo = 1e9, hi = -1e9;
            for (auto& g : slice(m, 0, xx))
                for (auto& pt : {g.first, g.second})
                    if (pt.y > 0.3 && pt.y < fob - 0.5 && pt.z < 1) { lo = std::min(lo, pt.z); hi = std::max(hi, pt.z); }
            r = std::max(r, hi - lo);
        }
        return r;
    };
    const double r2 = ripple(tessellate_hull(build_hull_surface(p), 240, 80), p);
    const double r4 = ripple(tessellate_hull(build_hull_surface(p4), 240, 80), p4);
    CHECK(r2 > 3e-4);
    CHECK(r4 < 1e-4);
    // ... while keeping the displacement and LCB at the design draft (underwater area matched
    // station by station).
    {
        auto be = make_cpu_backend(4);
        ShipRef ref{p.Lpp, p.B, p.Lpp / 2, 1.025};
        be->upload(tessellate_hull(build_hull_surface(p), 240, 80));
        HydroResult h2 = hydrostatic_table(*be, ref, {p.T})[0];
        be->upload(tessellate_hull(build_hull_surface(p4), 240, 80));
        HydroResult h4 = hydrostatic_table(*be, ref, {p.T})[0];
        CHECK(std::fabs(h4.volume / h2.volume - 1.0) < 2e-4);
        CHECK_NEAR(h4.B.x, h2.B.x, 0.05);
    }

    // 8. All stations but the stem fair within tolerance; midship has no inflection.
    HullFairing hf = fair_hull_sections(p, exact);
    FairingSummary sm = summarize(hf);
    CHECK(sm.stations == p.stations);
    CHECK(sm.not_applicable <= 1);
    int within = 0;
    for (auto& sec_i : hf.sections) within += sec_i.max_deviation <= exact.tolerance + 1e-9;
    CHECK(within >= p.stations - 1);
    CHECK(sm.max_deviation <= 2 * exact.tolerance);
    CHECK(hf.sections[p.stations / 2].q_composite.inflections == 0);
    CHECK(!hf.sections.back().ok || hf.sections.back().max_deviation <= 2 * so.tolerance);
}

static void test_offset_tables() {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path();
    auto write = [&](const char* name, const std::string& text) {
        const auto path = (dir / name).string();
        std::ofstream(path) << text;
        return path;
    };
    auto throws = [](const std::string& path, const char* expect) {
        try { read_offsets_csv(path); } catch (const std::invalid_argument& e) {
            return std::string(e.what()).find(expect) != std::string::npos;
        }
        return false;
    };

    // Round trip of a parametric table.
    TankerParams p;
    OffsetTable t = parametric_offset_table(p, 25);
    const auto rt = (dir / "hullcad_offsets_rt.csv").string();
    write_offsets_csv(t, rt);
    OffsetTable back = read_offsets_csv(rt);
    CHECK(back.stations.size() == t.stations.size());
    double err = 0;
    for (size_t i = 0; i < t.stations.size(); ++i)
        for (size_t j = 0; j < t.stations[i].size(); ++j) err = std::max(err, norm(back.stations[i][j] - t.stations[i][j]));
    CHECK(err < 1e-4);
    TankerParams q = params_for_table(back, TankerParams{});
    CHECK_NEAR(q.Lpp, p.Lpp, 1e-3);
    CHECK_NEAR(q.B, p.B, 1e-3);
    CHECK_NEAR(q.D, p.D, 1e-3);

    // Millimetres, semicolons, no station column (grouped by x), a deck-first station, shifted x.
    const auto mm = write("hullcad_offsets_mm.csv",
                          "# units = mm\nx;z;y\n"
                          "5000;0;0\n5000;0;1000\n5000;500;1500\n5000;2000;1600\n"
                          "15000;2000;1600\n15000;500;1500\n15000;0;1000\n15000;0;0\n"
                          "25000;0;0\n25000;0;1000\n25000;500;1500\n25000;2000;1600\n"
                          "35000;0;0\n35000;0;1000\n35000;500;1500\n35000;2000;1600\n");
    OffsetTable m = read_offsets_csv(mm);
    CHECK(m.stations.size() == 4);
    CHECK_NEAR(m.x_shift, -5.0, 1e-12);
    CHECK_NEAR(m.stations[1].front().x, 10.0, 1e-12);
    CHECK_NEAR(m.stations[1].front().y, 0.0, 0.0);   // reversed to keel-first
    CHECK_NEAR(m.stations[1].back().z, 2.0, 1e-12);
    CHECK_NEAR(describe(m).max_half_breadth, 1.6, 1e-12);

    CHECK(throws(write("hullcad_off_bad1.csv", "x,y\n0,0\n"), "x, y and z"));
    CHECK(throws(write("hullcad_off_bad2.csv", "x,y,z\n0,0,0\n0,1,0\n0,1,1\n"), "fewer than 4"));
    CHECK(throws(write("hullcad_off_bad3.csv", "x,y,z\n0,0,0\n0,abc,0\n"), ":3:"));
    CHECK(throws(write("hullcad_off_bad4.csv", "x,y,z\n0,1,0\n0,2,0\n0,2,1\n0,3,2\n"), "centreline"));

    // A measured-style table (25 points, +-1 mm, rounded to mm): fairing removes most inflections,
    // keeps the displacement, and keeps the flat blocks flat.
    OffsetTable noisy = parametric_offset_table(p, 25, 0.001, 0.001, 7);
    FairingOptions o;
    o.junction_continuity = 1;
    o.offset_accuracy = 0.001;
    HullFairing hf = fair_offset_table(noisy, o, p.points_per_section);
    FairingSummary sm = summarize(hf);
    CHECK(sm.not_applicable == 0);
    CHECK(sm.max_deviation <= o.tolerance + 1e-9);
    CHECK(sm.inflections_faired * 5 < sm.inflections_before);
    auto be = make_cpu_backend(4);
    ShipRef ref{p.Lpp, p.B, p.Lpp / 2, 1.025};
    be->upload(tessellate_hull(build_hull_surface(p), 240, 80));
    const double v0 = hydrostatic_table(*be, ref, {p.T})[0].volume;
    const TriMesh faired = tessellate_hull(build_faired_hull_surface(hf, p), 240, 80);
    be->upload(faired);
    CHECK(std::fabs(hydrostatic_table(*be, ref, {p.T})[0].volume / v0 - 1.0) < 5e-4);
    // Midship bottom: the faired surface is flat within 1 cm (flat-block criterion), the raw one is not.
    auto bottom_range = [&](const TriMesh& msh) {
        double lo = 1e9, hi = -1e9;
        for (double xx : {100.0, 120.0, 140.0})
            for (auto& g : slice(msh, 0, xx))
                for (auto& pt : {g.first, g.second})
                    if (pt.y > 0.3 && pt.y < 19.0 && pt.z < 1) { lo = std::min(lo, pt.z); hi = std::max(hi, pt.z); }
        return hi - lo;
    };
    // Raw: about 15 mm (beyond the 1 cm flat-block test); faired: a few mm, left by fitting each
    // station on its own (the curve-network fairing aligns stations along the ship).
    CHECK(bottom_range(faired) < 0.005);
    CHECK(bottom_range(tessellate_hull(build_offset_surface(hf), 240, 80)) > 0.01);
}

static void test_calendar() {
    CHECK(days_from_date("1970-01-01") == 0);
    CHECK(days_from_date("2027-03-02") == 20879);
    CHECK(date_from_days(20879) == "2027-03-02");
    CHECK(date_from_days(days_from_date("2028-02-29")) == "2028-02-29");
    for (long d = -800; d < 80000; d += 997) CHECK(days_from_date(date_from_days(d)) == d);
    CHECK(weekday_mon0(days_from_date("2026-11-16")) == 0);  // Monday
    CHECK(weekday_mon0(days_from_date("2027-03-02")) == 1);  // Tuesday
    auto bad = [](const char* s) {
        try { days_from_date(s); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    CHECK(bad("2027-02-29") && bad("2027-13-01") && bad("2027-3-2x") && bad("tomorrow"));
    YardProfile y;
    bool rejected = false;
    try { set_yard_option(y, "schedule.erection_start", "2027-02-30"); } catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);

    // Six-day weeks starting on Tuesday 2027-03-02: Sunday is skipped both ways.
    WorkCalendar c(days_from_date("2027-03-02"), 6);
    CHECK(c.date(0) == "2027-03-02");
    CHECK(c.date(4) == "2027-03-06");   // Saturday
    CHECK(c.date(5) == "2027-03-08");   // Monday
    CHECK(c.date(-1) == "2027-03-01");
    CHECK(c.date(-2) == "2027-02-27");  // Saturday before
    CHECK(c.date(11) == "2027-03-15");
    WorkCalendar sun(days_from_date("2027-03-07"), 5);  // starts on a Sunday -> Monday
    CHECK(sun.date(0) == "2027-03-08");
    CHECK(sun.date(5) == "2027-03-15");
}

static void test_yard_profile() {
    YardProfile y;
    set_yard_option(y, "transporter.units", "3");
    set_yard_option(y, "painting.enabled", "false");
    set_yard_option(y, "turnover.zones", "DB, DECK ,BHD");
    set_yard_option(y, "division.long_seams_y", "10.5, 5.5");
    CHECK(y.transporter.units == 3);
    CHECK(!y.painting.enabled);
    CHECK(y.turnover.zones.size() == 3 && y.turnover.zones[1] == "DECK");
    CHECK(y.division.long_seams_y.size() == 2);
    CHECK_NEAR(y.division.long_seams_y[1], 5.5, 0.0);
    CHECK_NEAR(y.transporter.combined_capacity(), 3 * 300 * 0.9, 1e-9);

    auto throws = [&](const char* k, const char* v) {
        try { set_yard_option(y, k, v); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    set_yard_option(y, "takeoff.coats", "4");  // moved key, still accepted
    CHECK(y.util_painting.coats == 4);
    CHECK(throws("transporter.colour", "red"));
    CHECK(throws("transporter.units", "2.5"));
    CHECK(throws("painting.enabled", "maybe"));
    CHECK(throws("assembly.crane_capacity", "300t"));

    // Every option survives a write -> load round trip.
    auto path = (std::filesystem::temp_directory_path() / "hullcad_yard_roundtrip.ini").string();
    y.name = "round trip";
    y.assembly.tandem_capacity = 432.5;
    write_yard_profile(y, path);
    YardProfile back = load_yard_profile(path, YardProfile{});
    for (auto& o : yard_options(y)) CHECK(get_yard_option(y, o.key) == get_yard_option(back, o.key));
    std::filesystem::remove(path);
}

static void test_tanker_division() {
    TankerParams p;
    TriMesh hull = tessellate_hull(build_hull_surface(p), 240, 80);
    TankerStructure st = default_tanker_structure(p);
    YardProfile y;
    ThreadPool pool(4);
    WeightModel wm = build_weight_model(hull, p, st, default_cargo_zones(p, st, y), default_end_zones(p, y), 0.25, pool);
    Division d = auto_divide(wm, p, st, y);
    CHECK(d.issues.empty());
    for (auto& s : d.issues) std::printf("  issue: %s\n", s.c_str());

    std::vector<double> members = st.bulkheads_x();
    for (double x : st.webs_x()) members.push_back(x);
    double sum = 0;
    int flat_db_mid = 0;
    for (auto& b : d.blocks) {
        CHECK(b.ok());
        CHECK(b.lift_weight <= y.transporter.combined_capacity() + y.weight.rigging_weight);
        sum += b.mass.w;
        // Midship double-bottom blocks (inside the parallel middle body) must come out flat.
        if (b.zone.rfind("DB", 0) == 0 && b.x0 > 0.35 * p.Lpp && b.x1 < 0.72 * p.Lpp) flat_db_mid += b.flat ? 1 : -100;
    }
    CHECK(flat_db_mid > 0);
    CHECK_NEAR(sum, wm.ring_mass(0, p.Lpp).w, 1.0);  // blocks cover the whole ship
    for (size_t i = 0; i + 1 < d.butts.size(); ++i) {
        double len = d.butts[i + 1] - d.butts[i];
        CHECK(len <= y.division.max_ring_length + 1e-9 && len >= y.division.min_ring_length - 1e-9);
    }
    for (size_t i = 1; i + 1 < d.butts.size(); ++i)
        for (double m : members) CHECK(std::fabs(d.butts[i] - m) >= y.division.min_butt_clearance);
    // Continuous longitudinals cap the ring at the stiffener length; the DP must reach that bound.
    int cargo_rings = 0;
    for (size_t i = 0; i + 1 < d.butts.size(); ++i) {
        double mid = 0.5 * (d.butts[i] + d.butts[i + 1]);
        if (mid > st.cargo_x0() && mid < st.cargo_x1()) ++cargo_rings;
    }
    double cap = std::min(y.division.max_ring_length, y.cutting.max_stiffener_length);
    CHECK(cargo_rings == static_cast<int>(std::ceil((st.cargo_x1() - st.cargo_x0()) / cap)));

    // A smaller transporter needs more rings; a disabled stage produces no checks.
    YardProfile small = y;
    small.transporter.unit_capacity = 80;
    Division ds = auto_divide(wm, p, st, small);
    CHECK(ds.issues.empty());
    CHECK(ds.butts.size() > d.butts.size());
    for (auto& b : ds.blocks) CHECK(b.transport_weight <= small.transporter.combined_capacity() + 1e-9);
    YardProfile no_paint = y;
    no_paint.painting.enabled = false;
    Block b0 = evaluate_block(wm, 0, d.butts[3], d.butts[4], no_paint);
    CHECK(std::none_of(b0.checks.begin(), b0.checks.end(), [](const Check& c) { return c.name.rfind("painting", 0) == 0; }));

    // Manual butt on a web frame, and a seam on a girder, are reported.
    CHECK(!evaluate_division(wm, p, st, {st.frames.x(65)}, y).issues.empty());
    YardProfile bad_seam = y;
    bad_seam.division.long_seams_y = {10.1};  // DB girder at y = 10.0
    bool seam_issue = false;
    for (auto& s : evaluate_division(wm, p, st, {}, bad_seam).issues) seam_issue |= s.find("DB girder") != std::string::npos;
    CHECK(seam_issue);

    // An impossible yard is diagnosed by name.
    YardProfile tiny = y;
    tiny.painting.cell_height = 5.0;
    Division di = auto_divide(wm, p, st, tiny);
    CHECK(di.butts.empty());
    bool named = false;
    for (auto& s : di.issues) named |= s.find("painting.size") != std::string::npos;
    CHECK(named);

    // Every block of a division carries its quantity takeoff; the ship totals are consistent.
    double plate = 0;
    for (auto& b : d.blocks) {
        CHECK(b.q.plate_area > 0);
        CHECK(b.q.fillet_length > 0);
        plate += b.q.plate_area;
    }
    UtilityReport ur = estimate_utilities(d, tank_volumes(hull, p, st), y);
    CHECK_NEAR(ur.quantities.plate_area, plate, 1e-6);
    CHECK(ur.tanks.cargo_m3 > 120000 && ur.tanks.cargo_m3 < 145000);  // 120k DWT tanker cargo capacity
    CHECK(ur.total.electricity_kwh > 0 && ur.total.air_nm3 > 0 && ur.total.water_m3 > 0);

    // Schedule: every unit of every utility lands on exactly one day (conservation).
    ProductionSchedule ps = schedule_blocks(d, y);
    DailyUtilityProfile sr = daily_utilities(ps, ur, y.capacity);
    for (auto& it : utility_items()) {
        double sum = 0;
        for (auto& day : sr.total) sum += day.*(it.member);
        CHECK_NEAR(sum, ur.total.*(it.member), 1e-9 * std::max(1.0, ur.total.*(it.member)));
    }
    // Erection starts in the key ring with a bottom block; stages follow each other before erection.
    const BlockSchedule* first = nullptr;
    for (auto& b : ps.blocks) if (b.erection_order == 1) first = &b;
    CHECK(first && first->erection_wd == 0);
    CHECK(first && d.butts[first->ring - 1] <= 0.35 * p.Lpp && 0.35 * p.Lpp < d.butts[first->ring]);
    CHECK(first && (first->zone.rfind("DB", 0) == 0 || first->zone.rfind("LOW", 0) == 0));
    for (size_t i = 0; i < ps.blocks.size(); ++i) {
        const auto& b = ps.blocks[i];
        auto sp = [&](Stage s) { return b.span[static_cast<int>(s)]; };
        CHECK(sp(Stage::Cutting).end == sp(Stage::Heating).start);
        CHECK(sp(Stage::Heating).end == sp(Stage::Welding).start);
        CHECK(sp(Stage::Welding).end == sp(Stage::Painting).start);
        CHECK(sp(Stage::Painting).end + y.schedule.paint_to_erection_days == b.erection_wd);
        CHECK((sp(Stage::Heating).days() > 0) == !d.blocks[i].flat);
        CHECK(b.erection_wd == static_cast<int>(std::floor((b.erection_order - 1) / y.schedule.erection_blocks_per_day)));
    }
    // Peaks: the daily maximum, converted to an hourly rate; capacity utilisation and overrun days.
    const PeakInfo& e = sr.peaks[0];
    double mx = 0;
    for (auto& day : sr.total) mx = std::max(mx, day.electricity_kwh);
    CHECK_NEAR(e.daily_peak, mx, 1e-9);
    CHECK_NEAR(e.peak_rate, mx / y.capacity.hours_per_day * y.capacity.peak_factor, 1e-9);
    YardProfile yc = y;
    yc.capacity.electricity_kw = e.peak_rate / 2;
    yc.capacity.base_electricity_kw = 100;
    DailyUtilityProfile sc = daily_utilities(schedule_blocks(d, yc), ur, yc.capacity);
    CHECK(sc.peaks[0].days_over > 0);
    CHECK_NEAR(sc.peaks[0].daily_peak, mx + 100 * y.capacity.hours_per_day, 1e-6);  // base load on every day
    double shares = 0;
    for (double v : sc.peaks[0].stage_share) shares += v;
    CHECK_NEAR(shares, 1.0, 1e-9);
}

int main() {
    test_frame_table();
    test_box_block_weights();
    test_yard_profile();
    test_calendar();
    test_fairing();
    test_offset_tables();
    test_box_takeoff_and_utilities();
    test_tanker_division();
    test_bspline();
    test_box_upright();
    test_box_heel_and_trim();
    test_tanker_mesh_and_determinism();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
