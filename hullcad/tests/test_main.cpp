// Self-contained test runner (no external framework).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "hullcad/bspline.hpp"
#include "hullcad/compute_backend.hpp"
#include "hullcad/hull_param.hpp"
#include "hullcad/hydrostatics.hpp"
#include "hullcad/mesh.hpp"

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

int main() {
    test_bspline();
    test_box_upright();
    test_box_heel_and_trim();
    test_tanker_mesh_and_determinism();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
