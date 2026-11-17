#include "hullcad/fairing/fairing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hullcad {

namespace {

constexpr double kInflectionEps = 1e-3;  // 1/m: curvature below this (radius > 1 km) counts as straight

double dist_to_line(Vec3 p, Vec3 a, Vec3 b) {
    const Vec3 d = b - a;
    const double L2 = dot(d, d);
    if (L2 <= 0) return norm(p - a);
    const double t = dot(p - a, d) / L2;
    return norm(p - (a + d * t));
}

// Least-squares line of pts[0..k]: centroid c and unit direction d (from pts[0] towards pts[k]).
void fit_line(const std::vector<Vec3>& pts, int k, Vec3& c, Vec3& d) {
    c = Vec3{};
    for (int i = 0; i <= k; ++i) c += pts[i];
    c = c * (1.0 / (k + 1));
    // Principal direction by power iteration on the scatter matrix.
    d = pts[k] - pts[0];
    if (norm(d) <= 0) d = Vec3{1, 0, 0};
    d = normalize(d);
    for (int it = 0; it < 30; ++it) {
        Vec3 nd{};
        for (int i = 0; i <= k; ++i) {
            const Vec3 q = pts[i] - c;
            nd += q * dot(q, d);
        }
        if (norm(nd) <= 0) break;
        d = normalize(nd);
    }
    if (dot(d, pts[k] - pts[0]) < 0) d = d * -1.0;
}

Vec3 project(Vec3 p, Vec3 c, Vec3 d) { return c + d * dot(p - c, d); }

// Largest k >= 2 such that pts[0..k] lie within tol of their least-squares line and the run is
// at least min_len long; 0 if there is no such run.
int lead_run(const std::vector<Vec3>& pts, double tol, double min_len) {
    int best = 0;
    for (int k = 2; k < static_cast<int>(pts.size()); ++k) {
        Vec3 c, d;
        fit_line(pts, k, c, d);
        bool ok = true;
        for (int i = 0; i <= k && ok; ++i) ok = norm(pts[i] - project(pts[i], c, d)) <= tol;
        if (!ok) break;
        if (norm(pts[k] - pts[0]) >= min_len) best = k;
    }
    return best;
}

std::vector<double> solve_dense(std::vector<double> A, std::vector<double> b, int n) {
    for (int c = 0; c < n; ++c) {
        int piv = c;
        for (int r = c + 1; r < n; ++r)
            if (std::fabs(A[r * n + c]) > std::fabs(A[piv * n + c])) piv = r;
        if (std::fabs(A[piv * n + c]) < 1e-300) throw std::runtime_error("fairing: singular system");
        if (piv != c) {
            for (int k = 0; k < n; ++k) std::swap(A[c * n + k], A[piv * n + k]);
            std::swap(b[c], b[piv]);
        }
        for (int r = c + 1; r < n; ++r) {
            const double f = A[r * n + c] / A[c * n + c];
            if (f == 0.0) continue;
            for (int k = c; k < n; ++k) A[r * n + k] -= f * A[c * n + k];
            b[r] -= f * b[c];
        }
    }
    std::vector<double> x(n);
    for (int r = n - 1; r >= 0; --r) {
        double s = b[r];
        for (int k = r + 1; k < n; ++k) s -= A[r * n + k] * x[k];
        x[r] = s / A[r * n + r];
    }
    return x;
}

double comp(const Vec3& v, int d) { return d == 0 ? v.x : d == 1 ? v.y : v.z; }

struct SegmentFit {
    BSplineCurve curve;
    double lambda = 0, max_dev = 0, rms = 0;
};

// Penalised least-squares cubic B-spline through P with optional end conditions:
// c = 1 fixes the end tangent direction t, c = 2 also makes the end curvature zero.
SegmentFit fit_segment(const std::vector<Vec3>& P, const std::vector<double>& w, Vec3 t0, int c0, Vec3 t1, int c1,
                       const FairingOptions& o) {
    const int p = 3;
    const int m = static_cast<int>(P.size()) - 1;
    if (m < 1) throw std::invalid_argument("fairing: need at least two points");
    const int nmin = std::max(4, 3 + c0 + c1);
    // Up to two control points per offset: the energy term regularises the extra freedom, which
    // lets sparse offsets place a curvature change between two offsets.
    const int n = std::max(nmin, std::min(2 * (m + 1), o.max_control_points));

    // Chord-length parameters and a uniform clamped knot vector.
    std::vector<double> u(m + 1, 0.0);
    for (int i = 1; i <= m; ++i) u[i] = u[i - 1] + norm(P[i] - P[i - 1]);
    const double total = u[m];
    for (auto& v : u) v /= total;
    std::vector<double> U(n + p + 1, 0.0);
    for (int i = n; i < n + p + 1; ++i) U[i] = 1.0;
    for (int i = 1; i < n - p; ++i) U[p + i] = static_cast<double>(i) / (n - p);

    // Control points as affine functions of the unknowns x: Q_j = c_j + sum_k x_k v_jk.
    struct Term { int k; Vec3 v; };
    std::vector<Vec3> cst(n);
    std::vector<std::vector<Term>> terms(n);
    int nx = 0;
    auto scalar = [&](int j, Vec3 base, Vec3 dir) { cst[j] = base; terms[j].push_back({nx++, dir}); };
    cst[0] = P[0];
    cst[n - 1] = P[m];
    std::vector<bool> fixed(n, false);
    fixed[0] = fixed[n - 1] = true;
    if (c0 >= 1) { scalar(1, P[0], t0); fixed[1] = true; }
    if (c0 >= 2) { scalar(2, P[0], t0); fixed[2] = true; }
    if (c1 >= 1) { scalar(n - 2, P[m], t1 * -1.0); fixed[n - 2] = true; }
    if (c1 >= 2) { scalar(n - 3, P[m], t1 * -1.0); fixed[n - 3] = true; }
    for (int j = 0; j < n; ++j) {
        if (fixed[j]) continue;
        terms[j].push_back({nx++, Vec3{1, 0, 0}});
        terms[j].push_back({nx++, Vec3{0, 1, 0}});
        terms[j].push_back({nx++, Vec3{0, 0, 1}});
    }

    // Normal equations of the data term (AtA, Atb) and the energy term (GtG, Gtd).
    std::vector<double> AtA(nx * nx, 0.0), Atb(nx, 0.0), GtG(nx * nx, 0.0), Gtd(nx, 0.0);
    auto accumulate = [&](const double* basis, int span, Vec3 target, double w, std::vector<double>& M,
                          std::vector<double>& r, bool is_data) {
        // Row (per coordinate): sum_j basis_j (c_j + sum v_jk x_k) -> a . x + const
        for (int d = 0; d < 3; ++d) {
            std::vector<std::pair<int, double>> a;
            double c = 0;
            for (int l = 0; l <= p; ++l) {
                const int j = span - p + l;
                c += basis[l] * comp(cst[j], d);
                for (auto& t : terms[j]) {
                    const double v = basis[l] * comp(t.v, d);
                    if (v != 0) a.push_back({t.k, v});
                }
            }
            const double rhs = is_data ? comp(target, d) - c : -c;  // data: a.x ~ P - c; energy: a.x ~ -c
            for (auto& [ki, vi] : a) {
                r[ki] += w * vi * rhs;
                for (auto& [kj, vj] : a) M[ki * nx + kj] += w * vi * vj;
            }
        }
    };
    double N[16];
    for (int i = 0; i <= m; ++i) {
        const int span = find_span(n - 1, p, u[i], U);
        basis_funs(span, u[i], p, U, N);
        accumulate(N, span, P[i], w[i], AtA, Atb, true);
    }
    const int r = std::clamp(o.energy_order, 1, 3);
    const double gx[3] = {-0.7745966692414834, 0.0, 0.7745966692414834}, gw[3] = {5.0 / 9, 8.0 / 9, 5.0 / 9};
    double ders[4 * 4];
    for (int span = p; span < n; ++span) {
        const double a = U[span], b = U[span + 1];
        if (b - a <= 0) continue;
        for (int g = 0; g < 3; ++g) {
            const double uu = 0.5 * (a + b) + 0.5 * (b - a) * gx[g];
            ders_basis_funs(span, uu, p, r, U, ders);
            accumulate(ders + r * (p + 1), span, Vec3{}, gw[g] * 0.5 * (b - a), GtG, Gtd, false);
        }
    }

    // Lambda scale so that the search range is independent of units and parameterisation.
    double ta = 0, tg = 0;
    for (int k = 0; k < nx; ++k) { ta += AtA[k * nx + k]; tg += GtG[k * nx + k]; }
    const double scale = tg > 0 ? ta / tg : 1.0;

    auto solve_for = [&](double lambda) {
        std::vector<double> H(nx * nx), g(nx);
        for (int k = 0; k < nx * nx; ++k) H[k] = AtA[k] + lambda * GtG[k];
        for (int k = 0; k < nx; ++k) g[k] = Atb[k] + lambda * Gtd[k];
        const auto x = solve_dense(std::move(H), std::move(g), nx);
        SegmentFit f;
        f.lambda = lambda;
        f.curve.degree = p;
        f.curve.knots = U;
        f.curve.ctrl.resize(n);
        for (int j = 0; j < n; ++j) {
            Vec3 q = cst[j];
            for (auto& t : terms[j]) q += t.v * x[t.k];
            f.curve.ctrl[j] = q;
        }
        double ss = 0;
        for (int i = 0; i <= m; ++i) {
            const double d = norm(f.curve.eval(u[i]) - P[i]);
            f.max_dev = std::max(f.max_dev, d);
            ss += d * d;
        }
        f.rms = std::sqrt(ss / (m + 1));
        return f;
    };

    // Largest lambda (fairest curve) whose deviation stays within the tolerance.
    double lo = -10, hi = 8;
    SegmentFit best = solve_for(scale * std::pow(10.0, lo));
    if (best.max_dev > o.tolerance) return best;  // cannot meet the tolerance: closest fit
    SegmentFit top = solve_for(scale * std::pow(10.0, hi));
    if (top.max_dev <= o.tolerance) return top;
    for (int it = 0; it < 40; ++it) {
        const double mid = 0.5 * (lo + hi);
        SegmentFit f = solve_for(scale * std::pow(10.0, mid));
        if (f.max_dev <= o.tolerance) { best = std::move(f); lo = mid; } else { hi = mid; }
    }
    return best;
}

// Arc-length table of a B-spline: (u, s) pairs.
std::vector<std::pair<double, double>> arc_table(const BSplineCurve& c, int n = 800) {
    std::vector<std::pair<double, double>> t;
    t.reserve(n + 1);
    const double u0 = c.umin(), u1 = c.umax();
    Vec3 prev = c.eval(u0);
    double s = 0;
    t.push_back({u0, 0.0});
    for (int i = 1; i <= n; ++i) {
        const double u = u0 + (u1 - u0) * i / n;
        const Vec3 q = c.eval(u);
        s += norm(q - prev);
        prev = q;
        t.push_back({u, s});
    }
    return t;
}

double u_at(const std::vector<std::pair<double, double>>& t, double s) {
    if (s <= 0) return t.front().first;
    if (s >= t.back().second) return t.back().first;
    auto it = std::lower_bound(t.begin(), t.end(), s, [](const auto& a, double v) { return a.second < v; });
    const auto& b = *it;
    const auto& a = *(it - 1);
    const double f = (s - a.second) / std::max(b.second - a.second, 1e-300);
    return a.first + f * (b.first - a.first);
}

}  // namespace

double FairedCurve::length() const {
    return norm(lead_b - lead_a) + arc_table(curve).back().second + norm(tail_b - tail_a);
}

Vec3 FairedCurve::point_at(double s) const {
    const double l0 = norm(lead_b - lead_a);
    if (s <= l0) return l0 > 0 ? lead_a + (lead_b - lead_a) * (s / l0) : lead_a;
    const auto t = arc_table(curve);
    const double lc = t.back().second;
    if (s <= l0 + lc) return curve.eval(u_at(t, s - l0));
    const double l1 = norm(tail_b - tail_a);
    return l1 > 0 ? tail_a + (tail_b - tail_a) * std::min(1.0, (s - l0 - lc) / l1) : tail_b;
}

std::vector<Vec3> FairedCurve::sample(int n) const {
    const auto t = arc_table(curve);
    const double l0 = norm(lead_b - lead_a), lc = t.back().second, l1 = norm(tail_b - tail_a);
    const double L = l0 + lc + l1;
    std::vector<Vec3> out;
    out.reserve(n);
    for (int k = 0; k < n; ++k) {
        const double s = L * k / (n - 1);
        if (s <= l0 && l0 > 0) out.push_back(lead_a + (lead_b - lead_a) * (s / l0));
        else if (s <= l0 + lc) out.push_back(curve.eval(u_at(t, s - l0)));
        else out.push_back(tail_a + (tail_b - tail_a) * std::min(1.0, (s - l0 - lc) / std::max(l1, 1e-300)));
    }
    out.front() = l0 > 0 ? lead_a : curve.eval(curve.umin());
    out.back() = l1 > 0 ? tail_b : curve.eval(curve.umax());
    return out;
}

FairedCurve fair_curve(const std::vector<Vec3>& pts, const FairingOptions& o) {
    const int m = static_cast<int>(pts.size()) - 1;
    if (m < 3) throw std::invalid_argument("fair_curve: need at least four points");
    // Straight runs at both ends (feature lines such as flat of bottom / flat of side).
    const double run_tol = std::max(o.straight_tolerance, 2.0 * o.offset_accuracy);
    int k = lead_run(pts, run_tol, o.min_straight_length);
    std::vector<Vec3> rev(pts.rbegin(), pts.rend());
    int j = m - lead_run(rev, run_tol, o.min_straight_length);
    // Hand the last junction_margin metres of each run to the curve.
    auto back_off = [&](int run_end, int run_start, int step) {
        const double keep = norm(pts[run_end] - pts[run_start]) - o.junction_margin;
        int i = run_end;
        while (i != run_start && norm(pts[i] - pts[run_start]) > keep) i -= step;
        return norm(pts[i] - pts[run_start]) >= o.min_straight_length ? i : run_start;
    };
    const int k0 = k, j0 = j;  // full straight runs
    if (o.junction_continuity >= 2) {  // a G1 junction stays at the tangent point
        if (k > 0) k = back_off(k, 0, 1);
        if (j < m) j = back_off(j, m, -1);
    }
    // The curve between the runs needs at least one offset of its own (sparse tables may have
    // a single offset on the bilge); otherwise fair the section as one curve.
    if (j - k < 2) { k = 0; j = m; }

    // Straight runs follow their least-squares lines (the measured points carry noise).
    Vec3 lc{}, ld{}, tc{}, td{};
    if (k > 0) fit_line(pts, k0, lc, ld);
    if (j < m) { fit_line(rev, m - j0, tc, td); td = td * -1.0; }
    const Vec3 la = k > 0 ? project(pts[0], lc, ld) : pts[0];
    const Vec3 lb = k > 0 ? project(pts[k], lc, ld) : pts[0];
    const Vec3 ta = j < m ? project(pts[j], tc, td) : pts[m];
    const Vec3 tb = j < m ? project(pts[m], tc, td) : pts[m];

    Vec3 t0{}, t1{};
    int c0 = 0, c1 = 0;
    if (k > 0) { t0 = ld; c0 = o.junction_continuity; }
    else if (norm(o.start_tangent) > 0) { t0 = normalize(o.start_tangent); c0 = 1; }
    if (j < m) { t1 = td; c1 = o.junction_continuity; }
    else if (norm(o.end_tangent) > 0) { t1 = normalize(o.end_tangent); c1 = 1; }

    std::vector<Vec3> mid(pts.begin() + k, pts.begin() + j + 1);
    mid.front() = lb;
    mid.back() = ta;
    // Offsets that lie on a detected straight run (inside the junction margin) weigh more.
    std::vector<double> w(mid.size(), 1.0);
    for (size_t i = 0; i < mid.size(); ++i) {
        const int g = k + static_cast<int>(i);
        if ((k > 0 && g <= k0) || (j < m && g >= j0)) w[i] = o.straight_weight;
    }
    SegmentFit f = fit_segment(mid, w, t0, c0, t1, c1, o);
    FairedCurve r;
    r.lead_a = la;
    r.lead_b = lb;
    r.tail_a = ta;
    r.tail_b = tb;
    r.curve = std::move(f.curve);
    r.lambda = f.lambda;
    // Deviation over all offsets: straight runs against their line, the rest against the fit.
    double ss = f.rms * f.rms * static_cast<double>(mid.size()), mx = f.max_dev;
    for (int i = 0; i < k; ++i) {
        const double d = dist_to_line(pts[i], r.lead_a, r.lead_b);
        mx = std::max(mx, d);
        ss += d * d;
    }
    for (int i = j + 1; i <= m; ++i) {
        const double d = dist_to_line(pts[i], r.tail_a, r.tail_b);
        mx = std::max(mx, d);
        ss += d * d;
    }
    r.max_deviation = mx;
    r.rms_deviation = std::sqrt(ss / (m + 1));
    return r;
}

std::vector<CurvatureSample> curvature_samples(const BSplineCurve& c, int n, Vec3 pn) {
    const auto t = arc_table(c);
    const double L = t.back().second;
    std::vector<CurvatureSample> out;
    out.reserve(n);
    Vec3 d[3];
    for (int k = 0; k < n; ++k) {
        const double s = L * k / (n - 1);
        c.derivs(u_at(t, s), 2, d);
        const double sp = norm(d[1]);
        CurvatureSample cs;
        cs.s = s;
        cs.p = d[0];
        cs.kappa = sp > 1e-14 ? dot(cross(d[1], d[2]), pn) / (sp * sp * sp) : 0.0;
        cs.n = sp > 1e-14 ? normalize(cross(pn, d[1])) : Vec3{};
        out.push_back(cs);
    }
    return out;
}

std::vector<CurvatureSample> curvature_samples(const FairedCurve& c, int n, Vec3 pn) {
    const auto t = arc_table(c.curve);
    const double l0 = norm(c.lead_b - c.lead_a), lc = t.back().second, l1 = norm(c.tail_b - c.tail_a);
    const double L = l0 + lc + l1;
    std::vector<CurvatureSample> out;
    out.reserve(n);
    Vec3 d[3];
    for (int k = 0; k < n; ++k) {
        const double s = L * k / (n - 1);
        CurvatureSample cs;
        cs.s = s;
        cs.kappa = 0;
        if (s < l0) {
            cs.p = c.lead_a + (c.lead_b - c.lead_a) * (s / l0);
            cs.n = normalize(cross(pn, c.lead_b - c.lead_a));
        } else if (s <= l0 + lc || l1 <= 0) {
            c.curve.derivs(u_at(t, s - l0), 2, d);
            const double sp = norm(d[1]);
            cs.p = d[0];
            cs.kappa = sp > 1e-14 ? dot(cross(d[1], d[2]), pn) / (sp * sp * sp) : 0.0;
            cs.n = sp > 1e-14 ? normalize(cross(pn, d[1])) : Vec3{};
        } else {
            cs.p = c.tail_a + (c.tail_b - c.tail_a) * std::min(1.0, (s - l0 - lc) / l1);
            cs.n = normalize(cross(pn, c.tail_b - c.tail_a));
        }
        out.push_back(cs);
    }
    return out;
}

CurveQuality curve_quality(const std::vector<CurvatureSample>& s) {
    CurveQuality q;
    int sign = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        if (std::fabs(s[i].kappa) > kInflectionEps) {
            const int sg = s[i].kappa > 0 ? 1 : -1;
            if (sign != 0 && sg != sign) ++q.inflections;
            sign = sg;
        }
        if (i > 0) {
            const double ds = s[i].s - s[i - 1].s;
            const double dk = s[i].kappa - s[i - 1].kappa;
            q.max_kappa_jump = std::max(q.max_kappa_jump, std::fabs(dk));
            q.variation += std::fabs(dk);
            if (ds > 0) q.energy += dk * dk / ds;
        }
    }
    return q;
}

namespace {

// n points equally spaced in arc length along the interpolant of pts.
std::vector<Vec3> resample(const std::vector<Vec3>& pts, int n) {
    const BSplineCurve c = interpolate(pts, 3);
    const auto t = arc_table(c, 2000);
    const double L = t.back().second;
    std::vector<Vec3> out;
    for (int k = 0; k < n; ++k) out.push_back(c.eval(u_at(t, L * k / (n - 1))));
    out.front() = pts.front();
    out.back() = pts.back();
    return out;
}

// Fairs one station. input: the points fitted; before_pts: the points whose interpolant is the
// unfaired section; offsets: unfaired section with the surface's point count (kept when the
// fairing is not applicable).
SectionFairing fair_station(double x, const std::vector<Vec3>& input, const std::vector<Vec3>& before_pts,
                            std::vector<Vec3> offsets, const FairingOptions& o) {
    const int npts = std::max(4, o.surface_points);
    SectionFairing s;
    s.x = x;
    s.offsets = std::move(offsets);
    s.composite = fair_curve(input, o);
    s.max_deviation = s.composite.max_deviation;
    s.ok = s.max_deviation <= 2.0 * o.tolerance;
    s.faired = s.ok ? s.composite.sample(npts) : resample(s.offsets, npts);
    s.faired.front().y = 0.0;  // keel on the centreline
    s.before = interpolate(before_pts, 3);
    s.after = interpolate(s.faired, 3);
    s.q_before = curve_quality(curvature_samples(s.before, 1200));
    s.q_after = curve_quality(curvature_samples(s.after, 1200));
    s.q_composite = curve_quality(curvature_samples(s.composite, 1200));
    return s;
}

}  // namespace

HullFairing fair_hull_sections(const TankerParams& p, const FairingOptions& opt, int dense_points) {
    HullFairing h;
    h.options = opt;
    FairingOptions o = opt;
    o.start_tangent = {0, 1, 0};  // symmetric section: horizontal at the keel
    o.end_tangent = {0, 0, 1};    // vertical at the deck edge
    for (double x : station_positions(p)) {
        const SectionShape shape = section_at(p, x);
        auto offsets = section_points(shape, x, p.D, p.points_per_section);
        const auto dense = section_points(shape, x, p.D, dense_points);
        auto before = offsets;
        h.sections.push_back(fair_station(x, dense, before, std::move(offsets), o));
    }
    return h;
}

HullFairing fair_offset_table(const OffsetTable& t, const FairingOptions& opt, int points_per_section) {
    HullFairing h;
    h.options = opt;
    for (auto& st : t.stations) {
        FairingOptions o = opt;
        // Horizontal at the keel when the section starts flat-ish (no keel knuckle); deck end free.
        const Vec3 d = st[1] - st[0];
        o.start_tangent = std::fabs(d.z) <= std::tan(20.0 * 3.14159265358979 / 180.0) * std::fabs(d.y) ? Vec3{0, 1, 0} : Vec3{};
        o.end_tangent = Vec3{};
        h.sections.push_back(fair_station(st.front().x, st, st, resample(st, points_per_section), o));
    }
    return h;
}

FairingSummary summarize(const HullFairing& f) {
    FairingSummary r;
    for (auto& s : f.sections) {
        ++r.stations;
        r.inflections_before += s.q_before.inflections;
        if (!s.ok) { ++r.not_applicable; continue; }
        r.inflections_faired += s.q_composite.inflections;
        r.max_deviation = std::max(r.max_deviation, s.max_deviation);
    }
    return r;
}

BSplineSurface build_faired_hull_surface(const HullFairing& f, const TankerParams&) {
    std::vector<std::vector<Vec3>> rows;
    for (auto& s : f.sections) rows.push_back(s.faired);
    return skin(rows, 3, 3);
}

BSplineSurface build_offset_surface(const HullFairing& f) {
    std::vector<std::vector<Vec3>> rows;
    for (auto& s : f.sections) rows.push_back(s.offsets);
    return skin(rows, 3, 3);
}

OffsetTable faired_offset_table(const HullFairing& f) {
    OffsetTable t;
    t.source = "faired";
    for (auto& s : f.sections) t.stations.push_back(s.faired);
    return t;
}

}  // namespace hullcad
