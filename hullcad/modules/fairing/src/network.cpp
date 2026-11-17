#include "hullcad/fairing/network.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace hullcad {

namespace {

// Dense polyline of a section (interpolant through its points) with cumulative girth.
struct Poly {
    std::vector<Vec3> p;
    std::vector<double> s;
};

Poly dense(const std::vector<Vec3>& pts, int n = 800) {
    Poly r;
    const BSplineCurve c = interpolate(pts, 3);
    for (int i = 0; i <= n; ++i) {
        const double u = c.umin() + (c.umax() - c.umin()) * i / n;
        r.p.push_back(c.eval(u));
        r.s.push_back(i == 0 ? 0.0 : r.s.back() + norm(r.p[i] - r.p[i - 1]));
    }
    return r;
}

// First crossing from the keel of the plane coordinate axis (1 = y, 2 = z) with value v.
bool cross(const Poly& poly, int axis, double v, Vec3& at, double& girth) {
    auto c = [&](const Vec3& q) { return axis == 1 ? q.y : q.z; };
    for (size_t i = 1; i < poly.p.size(); ++i) {
        const double a = c(poly.p[i - 1]) - v, b = c(poly.p[i]) - v;
        if ((a <= 0 && b > 0) || (a >= 0 && b < 0)) {
            const double t = a / (a - b);
            at = poly.p[i - 1] + (poly.p[i] - poly.p[i - 1]) * t;
            girth = poly.s[i - 1] + (poly.s[i] - poly.s[i - 1]) * t;
            return true;
        }
    }
    return false;
}

// Point of a faired longitudinal at x (the curve runs along the ship, x increasing).
bool at_x(const std::vector<Vec3>& samples, double x, Vec3& out) {
    for (size_t i = 1; i < samples.size(); ++i) {
        const double a = samples[i - 1].x, b = samples[i].x;
        if ((a <= x && x <= b) || (b <= x && x <= a)) {
            const double t = b != a ? (x - a) / (b - a) : 0.0;
            out = samples[i - 1] + (samples[i] - samples[i - 1]) * t;
            return true;
        }
    }
    return false;
}


// Longest interior run of a girth line (the parallel middle body) whose points lie within tol of
// their least-squares line; returns false if no run spans min_len.
bool middle_run(const std::vector<Vec3>& pts, double tol, double min_len, int& a, int& b, Vec3& c, Vec3& d) {
    const int n = static_cast<int>(pts.size());
    double best = 0;
    for (int i = 0; i + 2 < n; ++i) {
        for (int k = i + 2; k < n; ++k) {
            const std::vector<Vec3> sub(pts.begin() + i, pts.begin() + k + 1);
            Vec3 cc{}, dd{};
            // least-squares line (centroid + principal direction)
            for (auto& q : sub) cc += q;
            cc = cc * (1.0 / sub.size());
            dd = normalize(sub.back() - sub.front());
            for (int it = 0; it < 20; ++it) {
                Vec3 nd{};
                for (auto& q : sub) nd += (q - cc) * dot(q - cc, dd);
                if (norm(nd) <= 0) break;
                dd = normalize(nd);
            }
            if (dot(dd, sub.back() - sub.front()) < 0) dd = dd * -1.0;
            bool ok = true;
            for (auto& q : sub) ok = ok && norm(q - (cc + dd * dot(q - cc, dd))) <= tol;
            if (!ok) break;
            const double len = norm(sub.back() - sub.front());
            if (len >= min_len && len > best) { best = len; a = i; b = k; c = cc; d = dd; }
        }
    }
    return best > 0;
}

// Fairs one girth line: the parallel middle body (if any) becomes exactly straight and the aft
// and fore parts are faired into it with G2 junctions. Returns the faired point at each
// station's x (end stations kept).
std::vector<Vec3> fair_girth_line(std::vector<Vec3> line, const FairingOptions& fo, double run_tol, double min_run,
                                  bool& straight) {
    const int K = static_cast<int>(line.size());
    std::vector<Vec3> out = line;
    auto take = [&](const FairedCurve& fc, int from, int to) {
        const auto samples = fc.sample(4000);
        for (int i = std::max(from, 1); i <= std::min(to, K - 2); ++i) {
            Vec3 q = line[i];
            at_x(samples, line[i].x, q);
            q.x = line[i].x;
            out[i] = q;
        }
    };
    int a = 0, b = 0;
    Vec3 c{}, d{};
    straight = middle_run(line, run_tol, min_run, a, b, c, d);
    if (!straight) {
        take(fair_curve(line, fo), 0, K - 1);
        return out;
    }
    auto on_line = [&](double x) { return c + d * ((x - c.x) / (std::fabs(d.x) > 1e-12 ? d.x : 1e-12)); };
    for (int i = a; i <= b; ++i) line[i] = on_line(line[i].x);
    FairingOptions pf = fo;
    pf.straight_tolerance = 1e-7;  // the projected run is exactly straight; nothing else is
    pf.offset_accuracy = 0;
    if (a >= 1) {
        const std::vector<Vec3> aft(line.begin(), line.begin() + b + 1);
        if (aft.size() >= 4) take(fair_curve(aft, pf), 0, a);
    }
    if (b <= K - 2) {
        const std::vector<Vec3> fore(line.begin() + a, line.end());
        if (fore.size() >= 4) take(fair_curve(fore, pf), b, K - 1);
    }
    for (int i = std::max(a, 1); i <= std::min(b, K - 2); ++i) out[i] = line[i];
    return out;
}

}  // namespace

std::string LongitudinalCurve::name() const {
    char buf[48];
    std::snprintf(buf, sizeof buf, kind == 'W' ? "WL %.1f m" : "BL %.1f m", level);
    return buf;
}

NetworkOptions default_network(const TankerParams& p) {
    NetworkOptions o;
    for (double z = 1.0; z < p.D - 0.5; z += 2.0) o.waterlines.push_back(z);
    for (double y = 2.0; y < 0.5 * p.B - 1.5; y += 3.0) o.buttocks.push_back(y);
    return o;
}

BSplineCurve longitudinal_curve(const std::vector<Vec3>& pts) { return interpolate(pts, 3); }

NetworkResult fair_network(HullFairing& h, const NetworkOptions& o) {
    NetworkResult r;
    const int K = static_cast<int>(h.sections.size());
    for (double z : o.waterlines) { LongitudinalCurve c; c.kind = 'W'; c.level = z; r.curves.push_back(c); }
    for (double y : o.buttocks) { LongitudinalCurve c; c.kind = 'B'; c.level = y; r.curves.push_back(c); }
    auto plane_normal = [](const LongitudinalCurve& c) { return c.kind == 'W' ? Vec3{0, 0, 1} : Vec3{0, 1, 0}; };

    // Inspection curves cut from a set of sections.
    auto cut = [&](const std::vector<std::vector<Vec3>>& secs, std::vector<Vec3> LongitudinalCurve::*pts,
                   CurveQuality LongitudinalCurve::*q) {
        std::vector<Poly> polys;
        for (auto& sec : secs) polys.push_back(dense(sec));
        for (auto& c : r.curves) {
            (c.*pts).clear();
            for (int i = 0; i < K; ++i) {
                Vec3 at;
                double g;
                if (cross(polys[i], c.kind == 'W' ? 2 : 1, c.level, at, g)) (c.*pts).push_back(at);
            }
            c.*q = (c.*pts).size() >= 4 ? curve_quality(curvature_samples(interpolate(c.*pts, 3), 2000, plane_normal(c)))
                                        : CurveQuality{};
        }
    };
    auto sections = [&](bool faired) {
        std::vector<std::vector<Vec3>> v;
        for (auto& s : h.sections) v.push_back(faired ? s.faired : s.offsets);
        return v;
    };
    cut(sections(false), &LongitudinalCurve::raw, &LongitudinalCurve::q_raw);
    cut(sections(true), &LongitudinalCurve::sectioned, &LongitudinalCurve::q_sectioned);

    // Girth lines: point j of every station, faired along the ship; each station takes the
    // faired position at its own x (so the point stays in the station plane).
    const int npts = static_cast<int>(h.sections.front().faired.size());
    bool equal = K >= 4;
    for (auto& s : h.sections) equal = equal && static_cast<int>(s.faired.size()) == npts;
    if (!o.fair_girth_lines || !equal) {
        for (auto& c : r.curves) { c.faired = c.sectioned; c.q_faired = c.q_sectioned; }
        return r;
    }
    FairingOptions fo;
    fo.tolerance = o.tolerance;
    fo.offset_accuracy = o.offset_accuracy;
    fo.junction_continuity = 2;
    std::vector<std::vector<Vec3>> moved(K, std::vector<Vec3>(npts));
    const double run_tol = std::max(o.tolerance, 2.0 * o.offset_accuracy);
    const double min_run = o.min_middle_body * (h.sections.back().x - h.sections.front().x);
    for (int j = 0; j < npts; ++j) {
        std::vector<Vec3> line;
        for (auto& s : h.sections) line.push_back(s.faired[j]);
        bool straight = false;
        const auto f = fair_girth_line(line, fo, run_tol, min_run, straight);
        if (straight) ++r.straight_lines;
        for (int i = 0; i < K; ++i) {
            moved[i][j] = f[i];
            if (j == 0) moved[i][j].y = 0;  // keel line in the centre plane
        }
        ++r.girth_lines;
    }
    for (int i = 0; i < K; ++i)
        for (int j = 0; j < npts; ++j) {
            r.max_move = std::max(r.max_move, norm(moved[i][j] - h.sections[i].faired[j]));
            h.sections[i].faired[j] = moved[i][j];
        }
    for (auto& s : h.sections) {
        s.after = interpolate(s.faired, 3);
        s.q_after = curve_quality(curvature_samples(s.after, 1200));
    }
    cut(sections(true), &LongitudinalCurve::faired, &LongitudinalCurve::q_faired);
    return r;
}

int CurvatureMap::long_sign_changes(double threshold) const {
    int n = 0;
    for (int j = 0; j < nv; ++j) {
        int sign = 0;
        for (int i = 0; i < nu; ++i) {
            const double k = k_long[i * nv + j];
            if (std::fabs(k) < threshold) continue;
            const int sg = k > 0 ? 1 : -1;
            if (sign != 0 && sg != sign) ++n;
            sign = sg;
        }
    }
    return n;
}

CurvatureMap surface_curvature_map(const BSplineSurface& S, int nu, int nv) {
    CurvatureMap m;
    m.nu = nu;
    m.nv = nv;
    m.k_long.assign(nu * nv, 0.0);
    m.k_girth.assign(nu * nv, 0.0);
    const double u0 = S.knots_u[S.pu], u1 = S.knots_u[S.knots_u.size() - S.pu - 1];
    const double v0 = S.knots_v[S.pv], v1 = S.knots_v[S.knots_v.size() - S.pv - 1];
    m.x_min = S.eval(u0, 0.5 * (v0 + v1)).x;
    m.x_max = S.eval(u1, 0.5 * (v0 + v1)).x;
    const double hu = 1e-4 * (u1 - u0), hv = 1e-4 * (v1 - v0);
    for (int i = 0; i < nu; ++i) {
        const double u = u0 + (u1 - u0) * (i + 0.5) / nu;
        for (int j = 0; j < nv; ++j) {
            const double v = v0 + (v1 - v0) * (j + 0.5) / nv;
            const Vec3 p = S.eval(u, v);
            const Vec3 pu1 = S.eval(u + hu, v), pu0 = S.eval(u - hu, v);
            const Vec3 pv1 = S.eval(u, v + hv), pv0 = S.eval(u, v - hv);
            const Vec3 Su = (pu1 - pu0) * (0.5 / hu), Sv = (pv1 - pv0) * (0.5 / hv);
            const Vec3 Suu = (pu1 - p * 2.0 + pu0) * (1.0 / (hu * hu));
            const Vec3 Svv = (pv1 - p * 2.0 + pv0) * (1.0 / (hv * hv));
            const Vec3 nrm = cross(Su, Sv);
            if (norm(nrm) < 1e-14) continue;
            const Vec3 N = normalize(nrm);
            const double E = dot(Su, Su), G = dot(Sv, Sv);
            m.k_long[i * nv + j] = dot(Suu, N) / E;   // normal curvature along the ship
            m.k_girth[i * nv + j] = dot(Svv, N) / G;  // normal curvature along the girth
        }
    }
    return m;
}

}  // namespace hullcad
