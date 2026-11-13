#include "hullcad/bspline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hullcad {

int find_span(int n, int p, double u, const std::vector<double>& U) {
    if (u >= U[n + 1]) return n;
    if (u <= U[p]) return p;
    int low = p, high = n + 1, mid = (low + high) / 2;
    while (u < U[mid] || u >= U[mid + 1]) {
        if (u < U[mid]) high = mid; else low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

void basis_funs(int span, double u, int p, const std::vector<double>& U, double* N) {
    double left[16], right[16];
    N[0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - U[span + 1 - j];
        right[j] = U[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
}

void ders_basis_funs(int span, double u, int p, int nd, const std::vector<double>& U, double* ders) {
    const int P = p + 1;
    double ndu[16][16], a[2][16], left[16], right[16];
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - U[span + 1 - j];
        right[j] = U[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for (int j = 0; j <= p; ++j) ders[j] = ndu[j][p];
    for (int r = 0; r <= p; ++r) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= nd; ++k) {
            double d = 0.0;
            int rk = r - k, pk = p - k;
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            int j1 = (rk >= -1) ? 1 : -rk;
            int j2 = (r - 1 <= pk) ? k - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k * P + r] = d;
            std::swap(s1, s2);
        }
    }
    int r = p;
    for (int k = 1; k <= nd; ++k) {
        for (int j = 0; j <= p; ++j) ders[k * P + j] *= r;
        r *= (p - k);
    }
}

Vec3 BSplineCurve::eval(double u) const {
    int n = static_cast<int>(ctrl.size()) - 1;
    int span = find_span(n, degree, u, knots);
    double N[16];
    basis_funs(span, u, degree, knots, N);
    Vec3 c;
    for (int i = 0; i <= degree; ++i) c += ctrl[span - degree + i] * N[i];
    return c;
}

void BSplineCurve::derivs(double u, int nd, Vec3* out) const {
    int n = static_cast<int>(ctrl.size()) - 1;
    int span = find_span(n, degree, u, knots);
    double ders[16 * 16];
    nd = std::min(nd, degree);
    ders_basis_funs(span, u, degree, nd, knots, ders);
    for (int k = 0; k <= nd; ++k) {
        Vec3 c;
        for (int j = 0; j <= degree; ++j) c += ctrl[span - degree + j] * ders[k * (degree + 1) + j];
        out[k] = c;
    }
}

double BSplineCurve::curvature(double u) const {
    Vec3 d[3];
    derivs(u, 2, d);
    double s = norm(d[1]);
    if (s < 1e-14) return 0.0;
    return norm(cross(d[1], d[2])) / (s * s * s);
}

Vec3 BSplineSurface::eval(double u, double v) const {
    int su = find_span(nu - 1, pu, u, knots_u);
    int sv = find_span(nv - 1, pv, v, knots_v);
    double Nu[16], Nv[16];
    basis_funs(su, u, pu, knots_u, Nu);
    basis_funs(sv, v, pv, knots_v, Nv);
    Vec3 s;
    for (int k = 0; k <= pu; ++k) {
        Vec3 t;
        for (int l = 0; l <= pv; ++l) t += ctrl[(su - pu + k) * nv + (sv - pv + l)] * Nv[l];
        s += t * Nu[k];
    }
    return s;
}

std::vector<double> chord_length_params(const std::vector<Vec3>& pts) {
    std::vector<double> t(pts.size(), 0.0);
    double total = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        total += norm(pts[i] - pts[i - 1]);
        t[i] = total;
    }
    if (total <= 0.0) throw std::runtime_error("chord_length_params: degenerate point set");
    for (auto& v : t) v /= total;
    t.back() = 1.0;
    return t;
}

std::vector<double> averaged_knots(const std::vector<double>& params, int p) {
    const int n = static_cast<int>(params.size()) - 1;
    std::vector<double> U(n + p + 2, 0.0);
    for (int i = n + 1; i <= n + p + 1; ++i) U[i] = 1.0;
    for (int j = 1; j <= n - p; ++j) {
        double s = 0.0;
        for (int i = j; i < j + p; ++i) s += params[i];
        U[j + p] = s / p;
    }
    return U;
}

// Dense Gaussian elimination with partial pivoting; solves A X = B for 3 right-hand sides.
static std::vector<Vec3> solve_dense(std::vector<double> A, std::vector<Vec3> b, int n) {
    for (int c = 0; c < n; ++c) {
        int piv = c;
        for (int r = c + 1; r < n; ++r)
            if (std::fabs(A[r * n + c]) > std::fabs(A[piv * n + c])) piv = r;
        if (std::fabs(A[piv * n + c]) < 1e-300) throw std::runtime_error("solve_dense: singular matrix");
        if (piv != c) {
            for (int k = 0; k < n; ++k) std::swap(A[c * n + k], A[piv * n + k]);
            std::swap(b[c], b[piv]);
        }
        for (int r = c + 1; r < n; ++r) {
            double f = A[r * n + c] / A[c * n + c];
            if (f == 0.0) continue;
            for (int k = c; k < n; ++k) A[r * n + k] -= f * A[c * n + k];
            b[r] = b[r] - b[c] * f;
        }
    }
    std::vector<Vec3> x(n);
    for (int r = n - 1; r >= 0; --r) {
        Vec3 s = b[r];
        for (int k = r + 1; k < n; ++k) s = s - x[k] * A[r * n + k];
        x[r] = s * (1.0 / A[r * n + r]);
    }
    return x;
}

BSplineCurve interpolate(const std::vector<Vec3>& pts, int p,
                         const std::vector<double>& params, const std::vector<double>& U) {
    const int n = static_cast<int>(pts.size()) - 1;
    if (n < p) throw std::runtime_error("interpolate: not enough points for degree");
    std::vector<double> A((n + 1) * (n + 1), 0.0);
    double N[16];
    for (int i = 0; i <= n; ++i) {
        int span = find_span(n, p, params[i], U);
        basis_funs(span, params[i], p, U, N);
        for (int j = 0; j <= p; ++j) A[i * (n + 1) + span - p + j] = N[j];
    }
    BSplineCurve c;
    c.degree = p;
    c.knots = U;
    c.ctrl = solve_dense(std::move(A), pts, n + 1);
    return c;
}

BSplineCurve interpolate(const std::vector<Vec3>& pts, int p) {
    auto t = chord_length_params(pts);
    return interpolate(pts, p, t, averaged_knots(t, p));
}

BSplineSurface skin(const std::vector<std::vector<Vec3>>& sections, int pu, int pv) {
    const size_t K = sections.size();
    const size_t M = sections.at(0).size();
    for (auto& s : sections)
        if (s.size() != M) throw std::runtime_error("skin: sections must have equal point counts");

    // Common v parameters: average of each section's chord-length parameters.
    std::vector<double> tv(M, 0.0);
    for (auto& s : sections) {
        auto t = chord_length_params(s);
        for (size_t j = 0; j < M; ++j) tv[j] += t[j] / K;
    }
    tv.front() = 0.0; tv.back() = 1.0;
    auto Uv = averaged_knots(tv, pv);

    // Common u parameters: average over v-index of chord lengths across sections.
    std::vector<double> tu(K, 0.0);
    for (size_t j = 0; j < M; ++j) {
        std::vector<Vec3> col(K);
        for (size_t k = 0; k < K; ++k) col[k] = sections[k][j];
        auto t = chord_length_params(col);
        for (size_t k = 0; k < K; ++k) tu[k] += t[k] / M;
    }
    tu.front() = 0.0; tu.back() = 1.0;
    auto Uu = averaged_knots(tu, pu);

    // Interpolate each section in v, then each control-point column in u.
    std::vector<std::vector<Vec3>> Q(K);
    for (size_t k = 0; k < K; ++k) Q[k] = interpolate(sections[k], pv, tv, Uv).ctrl;

    BSplineSurface S;
    S.pu = pu; S.pv = pv;
    S.knots_u = Uu; S.knots_v = Uv;
    S.nu = static_cast<int>(K); S.nv = static_cast<int>(M);
    S.ctrl.resize(K * M);
    for (size_t j = 0; j < M; ++j) {
        std::vector<Vec3> col(K);
        for (size_t k = 0; k < K; ++k) col[k] = Q[k][j];
        auto c = interpolate(col, pu, tu, Uu);
        for (size_t k = 0; k < K; ++k) S.ctrl[k * M + j] = c.ctrl[k];
    }
    return S;
}

}  // namespace hullcad
