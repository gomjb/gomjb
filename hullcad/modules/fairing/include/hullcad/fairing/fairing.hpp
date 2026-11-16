#pragma once
// Curve fairing (순정) for hull sections.
//
// A fair curve is not merely C2 (a cubic B-spline always is): its curvature must change
// smoothly, without unnecessary oscillation or sign changes. Fairing therefore fits a cubic
// B-spline to the offsets while minimising the curvature-variation energy
//
//     sum_i |C(u_i) - P_i|^2  +  lambda * integral |C'''(u)|^2 du
//
// and picks the largest lambda whose maximum deviation from the offsets stays within the
// tolerance. Straight runs at either end (flat of bottom, flat of side) are detected and kept
// exactly straight; the curve between them joins them with G1 (tangent) or G2 (tangent and
// zero curvature, i.e. curvature-continuous) continuity.
#include <string>
#include <vector>

#include "hullcad/cad/bspline.hpp"
#include "hullcad/cad/hull_param.hpp"

namespace hullcad {

struct FairingOptions {
    double tolerance = 0.003;        // m, maximum deviation from the offsets
    int junction_continuity = 2;     // 1 = G1, 2 = G2 where the curve meets a straight run
    int energy_order = 3;            // 3: curvature variation (C'''), 2: bending (C'')
    int max_control_points = 48;
    double straight_weight = 400.0;  // weight of offsets on a straight run handed to the curve:
                                     // keeps the flat flat and lets the bilge absorb the deviation
    double straight_tolerance = 5e-4;  // m, offsets within this of a line count as straight
    double offset_accuracy = 0;        // m, expected error of the offsets (measured / entered);
                                       // straight runs are detected within max(straight_tolerance,
                                       // 2 x offset_accuracy) of their least-squares line
    double min_straight_length = 0.5;  // m
    double junction_margin = 1.0;      // m of each straight run handed to the curve, so a G2
                                       // transition can start inside the run (the true flat-of-
                                       // bottom line is where the curvature starts to rise)
    Vec3 start_tangent{};  // direction at the first point when no straight run starts there (0 = free)
    Vec3 end_tangent{};    // direction at the last point when no straight run ends there (0 = free)
    Vec3 plane_normal{1, 0, 0};  // curves lie in planes normal to this (signed curvature sign)
};

// Faired composite: leading straight run, faired B-spline, trailing straight run.
struct FairedCurve {
    Vec3 lead_a, lead_b;  // lead_a == lead_b: no leading run
    BSplineCurve curve;   // from lead_b to tail_a
    Vec3 tail_a, tail_b;  // tail_a == tail_b: no trailing run
    double lambda = 0;
    double max_deviation = 0, rms_deviation = 0;  // at the offsets, m
    double length() const;
    Vec3 point_at(double s) const;                // by arc length from the start
    std::vector<Vec3> sample(int n) const;        // n points equally spaced in arc length
};

// Fair one ordered point set (e.g. a section from keel to deck edge).
FairedCurve fair_curve(const std::vector<Vec3>& pts, const FairingOptions& opt);

// Fairness measures of a curve given as a dense polyline-free evaluator.
struct CurveQuality {
    int inflections = 0;          // sign changes of the signed curvature (|kappa| > 1e-3 1/m)
    double energy = 0;            // integral (d kappa / ds)^2 ds
    double max_kappa_jump = 0;    // largest curvature jump between neighbouring samples, 1/m
};
struct CurvatureSample {
    double s;       // arc length
    Vec3 p;         // point
    Vec3 n;         // unit normal (in plane), pointing to the comb side
    double kappa;   // signed curvature, 1/m
};
// Signed curvature along a B-spline / along a faired composite, sampled by arc length.
std::vector<CurvatureSample> curvature_samples(const BSplineCurve& c, int n, Vec3 plane_normal = {1, 0, 0});
std::vector<CurvatureSample> curvature_samples(const FairedCurve& c, int n, Vec3 plane_normal = {1, 0, 0});
CurveQuality curve_quality(const std::vector<CurvatureSample>& s);

// ---- Hull sections

struct SectionFairing {
    double x = 0;
    std::vector<Vec3> offsets;   // the offsets the surface would be built from without fairing
    std::vector<Vec3> faired;    // faired offsets (same count) used to build the surface
    FairedCurve composite;       // faired section
    BSplineCurve before, after;  // section curves the surface actually carries (interpolants)
    CurveQuality q_before, q_after;
    CurveQuality q_composite;    // the faired section itself
    double max_deviation = 0;    // faired composite vs the dense design offsets, m
    bool ok = true;              // false: tolerance missed by more than 2x (e.g. the stem section,
                                 // whose bottom radius is millimetres); the offsets are kept
};

struct HullFairing {
    FairingOptions options;
    std::vector<SectionFairing> sections;
};

// Fairs every station of the parametric hull. dense_points: offsets per station fed to the
// fairing (the surface still uses p.points_per_section per station).
HullFairing fair_hull_sections(const TankerParams& p, const FairingOptions& opt, int dense_points = 481);
// Skinned surface through the faired sections (offsets where fairing was not applicable).
BSplineSurface build_faired_hull_surface(const HullFairing& f, const TankerParams& p);

// Sums over the stations, for reports.
struct FairingSummary {
    int stations = 0, not_applicable = 0;
    int inflections_before = 0, inflections_faired = 0;
    double max_deviation = 0;  // over the applicable stations, m
};
FairingSummary summarize(const HullFairing& f);

}  // namespace hullcad
