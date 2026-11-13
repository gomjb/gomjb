#include "hullcad/hull_param.hpp"

#include <algorithm>
#include <cmath>

namespace hullcad {

namespace {

constexpr double kPi = 3.14159265358979323846;

// C2-continuous ramp from 0 (xi <= a) to 1 (xi >= b).
double smoother(double a, double b, double xi) {
    double t = std::clamp((xi - a) / (b - a), 0.0, 1.0);
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

}  // namespace

SectionShape section_at(const TankerParams& p, double x) {
    const double xi = x / p.Lpp;
    const double hb = 0.5 * p.B;
    // 1 at the end, 0 in the parallel middle body.
    const double wa = 1.0 - smoother(0.0, p.xi_pmb_aft, xi);

    SectionShape s{};
    // Deck half-breadth: run to a transom aft.
    double b_aft = hb * (1.0 - (1.0 - p.transom_half_breadth_ratio) * std::pow(wa, 1.6));
    // Full, rounded bow planform: super-ellipse quadrant from the shoulder to the stem.
    double tb = std::clamp((xi - p.xi_bow_shoulder) / (1.0 - p.xi_bow_shoulder), 0.0, 1.0);
    double b_fwd = std::max(p.stem_half_breadth,
                            hb * std::pow(std::max(0.0, 1.0 - std::pow(tb, p.bow_planform_r)), 1.0 / p.bow_planform_q));
    s.b = std::min(b_aft, b_fwd);

    // Keel line: stern cut-up and forefoot rise.
    s.zk = p.stern_cutup_height * (1.0 - smoother(0.0, p.xi_cutup_end, xi)) +
           p.forefoot_rise * smoother(0.965, 1.0, xi);

    // Flat of bottom.
    const double R = p.bilge_radius;
    double fob = smoother(p.xi_fob_aft, p.xi_pmb_aft, xi) * (1.0 - smoother(p.xi_bow_corner - 0.04, p.xi_fob_fwd, xi));
    s.fb = std::max(0.0, std::min((hb - R) * fob, s.b - R));

    // Bilge corner grows to span the whole section toward the ends.
    const double wfc = smoother(p.xi_bow_corner, 1.0, xi);
    double w_end = std::max(std::pow(wa, 0.8), wfc);
    s.rz = R + (p.D - s.zk - R) * w_end;
    s.ry = s.b - s.fb;
    s.n = 2.0 + (p.n_aft - 2.0) * wa + (p.n_fwd - 2.0) * wfc;
    return s;
}

std::vector<Vec3> section_points(const SectionShape& s, double x, double D, int npts) {
    // Dense polyline, then resample uniformly by girth length.
    std::vector<Vec3> poly;
    poly.push_back({x, 0.0, s.zk});
    if (s.fb > 1e-9) poly.push_back({x, s.fb, s.zk});
    const int ncorner = 400;
    for (int i = 1; i <= ncorner; ++i) {
        double t = 0.5 * kPi * i / ncorner;
        double y = s.fb + s.ry * std::pow(std::sin(t), 2.0 / s.n);
        double z = s.zk + s.rz - s.rz * std::pow(std::cos(t), 2.0 / s.n);
        poly.push_back({x, y, z});
    }
    if (D - poly.back().z > 1e-9) poly.push_back({x, s.b, D});

    std::vector<double> cum(poly.size(), 0.0);
    for (size_t i = 1; i < poly.size(); ++i) cum[i] = cum[i - 1] + norm(poly[i] - poly[i - 1]);
    const double L = cum.back();

    std::vector<Vec3> out;
    out.reserve(npts);
    size_t seg = 1;
    for (int k = 0; k < npts; ++k) {
        double target = L * k / (npts - 1);
        while (seg + 1 < poly.size() && cum[seg] < target) ++seg;
        double t = (target - cum[seg - 1]) / std::max(cum[seg] - cum[seg - 1], 1e-300);
        t = std::clamp(t, 0.0, 1.0);
        out.push_back(poly[seg - 1] + (poly[seg] - poly[seg - 1]) * t);
    }
    out.front() = poly.front();
    out.back() = poly.back();
    return out;
}

std::vector<double> station_positions(const TankerParams& p) {
    // Cosine clustering: denser at the ends where the shape changes fastest.
    std::vector<double> xs(p.stations);
    for (int i = 0; i < p.stations; ++i) {
        double t = static_cast<double>(i) / (p.stations - 1);
        double c = 0.5 * (1.0 - std::cos(kPi * t));
        xs[i] = p.Lpp * (0.55 * c + 0.45 * t);
    }
    return xs;
}

std::vector<std::vector<Vec3>> offsets(const TankerParams& p) {
    std::vector<std::vector<Vec3>> rows;
    for (double x : station_positions(p))
        rows.push_back(section_points(section_at(p, x), x, p.D, p.points_per_section));
    return rows;
}

BSplineSurface build_hull_surface(const TankerParams& p) { return skin(offsets(p), 3, 3); }

}  // namespace hullcad
