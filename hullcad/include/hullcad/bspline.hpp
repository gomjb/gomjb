#pragma once
// Non-rational B-spline curves and surfaces (algorithms after Piegl & Tiller, "The NURBS Book").
#include <vector>

#include "hullcad/vec3.hpp"

namespace hullcad {

struct BSplineCurve {
    int degree = 3;
    std::vector<double> knots;  // size = ctrl.size() + degree + 1
    std::vector<Vec3> ctrl;

    double umin() const { return knots[degree]; }
    double umax() const { return knots[knots.size() - degree - 1]; }
    Vec3 eval(double u) const;
    // out[0..nd] = C(u), C'(u), ..., C^(nd)(u); nd <= degree
    void derivs(double u, int nd, Vec3* out) const;
    double curvature(double u) const;
};

struct BSplineSurface {
    int pu = 3, pv = 3;
    std::vector<double> knots_u, knots_v;
    int nu = 0, nv = 0;       // number of control points in u / v
    std::vector<Vec3> ctrl;   // ctrl[i * nv + j]

    Vec3 eval(double u, double v) const;
};

// Knot span index (A2.1). n = number of control points - 1.
int find_span(int n, int p, double u, const std::vector<double>& U);
// Non-vanishing basis functions N[0..p] at u (A2.2).
void basis_funs(int span, double u, int p, const std::vector<double>& U, double* N);
// Basis functions and derivatives ders[k][j], k = 0..nd (A2.3). ders is (nd+1) x (p+1) row-major.
void ders_basis_funs(int span, double u, int p, int nd, const std::vector<double>& U, double* ders);

std::vector<double> chord_length_params(const std::vector<Vec3>& pts);
std::vector<double> averaged_knots(const std::vector<double>& params, int degree);

// Global interpolation through pts at the given parameters and knot vector (A9.1).
BSplineCurve interpolate(const std::vector<Vec3>& pts, int degree,
                         const std::vector<double>& params, const std::vector<double>& knots);
BSplineCurve interpolate(const std::vector<Vec3>& pts, int degree);

// Skinned surface through a family of sections (u direction = across sections,
// v direction = along each section). All sections must have the same point count.
BSplineSurface skin(const std::vector<std::vector<Vec3>>& sections, int pu, int pv);

}  // namespace hullcad
