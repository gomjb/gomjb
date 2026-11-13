#include "hullcad/hydrostatics.hpp"

#include <cmath>
#include <stdexcept>

namespace hullcad {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}

WaterPlane make_plane(double draft, double heel_deg, double trim_deg, double x_ref) {
    const double ph = heel_deg * kDeg, th = trim_deg * kDeg;
    WaterPlane wp;
    // Earth "up" seen from the ship: leans to port for starboard-down heel, aft for bow-down trim.
    wp.n = {-std::sin(th), std::sin(ph) * std::cos(th), std::cos(ph) * std::cos(th)};
    Vec3 p{x_ref, 0.0, draft};
    wp.h = dot(wp.n, p);
    wp.o = p;
    Vec3 ex{1.0, 0.0, 0.0};
    wp.e1 = normalize(ex - wp.n * dot(ex, wp.n));
    wp.e2 = cross(wp.n, wp.e1);
    return wp;
}

WaterPlane plane_with_offset(const WaterPlane& base, double h) {
    WaterPlane wp = base;
    wp.o = base.o + base.n * (h - base.h);
    wp.h = h;
    return wp;
}

HydroResult derive(const HydroAccum& a, const WaterPlane& wp, const ShipRef& ref,
                   double draft, double heel, double trim) {
    HydroResult r;
    r.draft = draft; r.heel = heel; r.trim = trim;
    r.volume = a.V;
    r.displacement = a.V * ref.rho;
    if (a.V > 0) r.B = wp.o + Vec3{a.Mx, a.My, a.Mz} * (1.0 / a.V);
    r.Awp = a.A;
    if (a.A > 0) {
        double uc = a.Su / a.A, tc = a.St / a.A;
        r.F = wp.o + wp.e1 * uc + wp.e2 * tc;
        r.I_L = a.Suu - a.A * uc * uc;
        r.I_T = a.Stt - a.A * tc * tc;
    }
    if (a.V > 0) {
        r.BM_T = r.I_T / a.V;
        r.BM_L = r.I_L / a.V;
        r.KM_T = r.B.z + r.BM_T;
        r.KM_L = r.B.z + r.BM_L;
        r.Cb = a.V / (ref.Lpp * ref.B * draft);
        r.MCT = ref.rho * r.I_L / (100.0 * ref.Lpp);  // = Disp * BM_L / (100 L), with GM_L ~ BM_L
    }
    r.TPC = ref.rho * a.A / 100.0;
    r.Cw = a.A / (ref.Lpp * ref.B);
    return r;
}

std::vector<HydroResult> hydrostatic_table(ComputeBackend& be, const ShipRef& ref,
                                           const std::vector<double>& drafts) {
    std::vector<WaterPlane> planes;
    for (double T : drafts) planes.push_back(make_plane(T, 0.0, 0.0, ref.x_ref));
    std::vector<HydroAccum> acc(planes.size());
    be.evaluate(planes, acc);
    std::vector<HydroResult> out;
    for (size_t i = 0; i < planes.size(); ++i)
        out.push_back(derive(acc[i], planes[i], ref, drafts[i], 0.0, 0.0));
    return out;
}

std::vector<GZPoint> gz_curve(ComputeBackend& be, const ShipRef& ref, double volume, Vec3 G,
                              const std::vector<double>& heels_deg, double trim_deg,
                              double tol_rel, int max_iter) {
    const size_t n = heels_deg.size();
    std::vector<WaterPlane> cur(n);
    std::vector<HydroAccum> acc(n);
    std::vector<bool> done(n, false);
    std::vector<GZPoint> out(n);
    // Initial guess: draft of a box with Cb = 0.8 holding the target volume.
    const double T0 = volume / (0.8 * ref.Lpp * ref.B);
    for (size_t i = 0; i < n; ++i) {
        cur[i] = make_plane(T0, heels_deg[i], trim_deg, ref.x_ref);
        out[i].heel = heels_deg[i];
    }
    // Newton on h for all unconverged heel angles at once: h += (V0 - V) / Awp.
    std::vector<WaterPlane> batch;
    std::vector<HydroAccum> res;
    std::vector<size_t> idx;
    for (int it = 1; it <= max_iter; ++it) {
        batch.clear(); idx.clear();
        for (size_t i = 0; i < n; ++i)
            if (!done[i]) { batch.push_back(cur[i]); idx.push_back(i); }
        if (batch.empty()) break;
        res.assign(batch.size(), HydroAccum{});
        be.evaluate(batch, res);
        for (size_t k = 0; k < batch.size(); ++k) {
            const size_t i = idx[k];
            acc[i] = res[k];
            out[i].iterations = it;
            const double err = volume - res[k].V;
            if (std::fabs(err) <= tol_rel * volume) { done[i] = true; continue; }
            if (res[k].A <= 0) throw std::runtime_error("gz_curve: waterplane lost (hull dry or sunk)");
            double dh = err / res[k].A;
            const double lim = 0.25 * ref.B;  // safeguard against wild first steps
            if (dh > lim) dh = lim; else if (dh < -lim) dh = -lim;
            cur[i] = plane_with_offset(cur[i], cur[i].h + dh);
        }
    }
    for (size_t i = 0; i < n; ++i) {
        if (!done[i]) throw std::runtime_error("gz_curve: Newton iteration did not converge");
        const WaterPlane& wp = cur[i];
        const Vec3 B = wp.o + Vec3{acc[i].Mx, acc[i].My, acc[i].Mz} * (1.0 / acc[i].V);
        // -e2 is the horizontal transverse direction toward the immersed (starboard) side.
        out[i].GZ = dot(B - G, wp.e2 * -1.0);
        out[i].draft = (wp.h - wp.n.x * ref.x_ref) / wp.n.z;
    }
    return out;
}

}  // namespace hullcad
