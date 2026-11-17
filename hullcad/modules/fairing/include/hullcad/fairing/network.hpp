#pragma once
// Curve-network fairing (곡선망 순정) and surface curvature checks.
//
// Faired sections alone can still disagree from station to station: each section is fitted on
// its own, so a flat bottom may sit a few millimetres higher at one station than at the next
// (offsets measured or entered station by station), and the surface shows stripes along the
// ship. The network fairing fairs the surface net in the other direction too: the girth lines
// (the points at the same girth position on every station) are faired along the ship. The
// parallel middle body (the longest run of a girth line within the tolerance of a straight line)
// becomes exactly straight, and the run and entrance are faired into it with G2 junctions. Each
// station takes the faired positions at its own x, so sections and girth lines form one fair net.
//
// Waterlines (z = const) and buttocks (y = const) are the classic inspection curves: they are
// cut from the unfaired, the section-faired and the network-faired sections and compared by the
// total variation of their curvature.
#include <string>
#include <vector>

#include "hullcad/cad/bspline.hpp"
#include "hullcad/fairing/fairing.hpp"

namespace hullcad {

struct NetworkOptions {
    std::vector<double> waterlines;  // inspection: z levels, m
    std::vector<double> buttocks;    // inspection: y offsets (port), m
    double tolerance = 0.003;        // m, maximum move of a section point by the girth-line fairing
    double offset_accuracy = 0;      // m, as FairingOptions::offset_accuracy
    double min_middle_body = 0.1;    // shortest straight run of a girth line, fraction of the length
    bool fair_girth_lines = true;    // false: only cut the inspection curves (no section moves)
};
// Waterlines every 2 m from 1 m to D - 1 m, buttocks every 3 m from 2 m to B/2 - 2 m.
NetworkOptions default_network(const TankerParams& p);

struct LongitudinalCurve {
    char kind = 'W';      // 'W' waterline (z = level), 'B' buttock (y = level)
    double level = 0;
    std::vector<Vec3> raw;        // cut from the unfaired sections
    std::vector<Vec3> sectioned;  // cut from the section-faired sections
    std::vector<Vec3> faired;     // cut from the network-faired sections (= sectioned if not applied)
    CurveQuality q_raw, q_sectioned, q_faired;
    std::string name() const;
};

struct NetworkResult {
    std::vector<LongitudinalCurve> curves;
    int girth_lines = 0;       // girth lines faired (0: not applied)
    int straight_lines = 0;    // of which with a straight middle body
    double max_move = 0;       // largest move of a section point, m
};

// Fairs the girth lines and updates h.sections[i].faired (and .after / .q_after) in place.
// Needs the same number of points on every station (always true for fair_offset_table /
// fair_hull_sections); otherwise only the inspection curves are cut.
NetworkResult fair_network(HullFairing& h, const NetworkOptions& o);
// Interpolant of a longitudinal's points (for plots).
BSplineCurve longitudinal_curve(const std::vector<Vec3>& pts);

// Normal curvatures of a surface on a grid (u along the ship, v along the girth).
struct CurvatureMap {
    int nu = 0, nv = 0;
    std::vector<double> k_long, k_girth;  // [i * nv + j], 1/m
    double x_min = 0, x_max = 0;
    // Sign changes of the longitudinal curvature along u, summed over all girth lines
    // (|k| above threshold): stripes when neighbouring stations disagree.
    int long_sign_changes(double threshold = 2e-4) const;
};
CurvatureMap surface_curvature_map(const BSplineSurface& S, int nu = 240, int nv = 60);

}  // namespace hullcad
