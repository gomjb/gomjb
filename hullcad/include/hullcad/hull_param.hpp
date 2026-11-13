#pragma once
// Simplified parametric hull for a ~120k DWT crude oil tanker (Aframax / LR2 class).
//
// Each station is a "rounded-box" section: flat bottom, a super-ellipse bilge corner and a
// vertical side. Midship the corner is an exact circle (n = 2, ry = rz = bilge radius); toward
// the ends the corner grows until it spans the whole section, giving V-sections aft and
// U-sections forward. Bulbous bow, stern skeg/boss and above-water flare are NOT modelled yet.
//
// Coordinates: x from AP (0) to FP (Lpp), y to port (half-breadth >= 0), z up from baseline.
#include <vector>

#include "hullcad/bspline.hpp"

namespace hullcad {

struct TankerParams {
    double Lpp = 239.0;   // m
    double B = 44.0;      // m, moulded breadth
    double D = 21.0;      // m, moulded depth
    double T = 15.0;      // m, design draft
    double bilge_radius = 2.4;

    double xi_pmb_aft = 0.33;   // aft end of the parallel middle body, as fraction of Lpp
    double xi_fob_aft = 0.12;   // flat-of-bottom fade-out positions
    double xi_fob_fwd = 0.93;

    double transom_half_breadth_ratio = 0.60;  // deck half-breadth at AP / (B/2)
    double stem_half_breadth = 0.4;            // m, at FP
    double xi_bow_shoulder = 0.74;             // forward shoulder: full breadth aft of this
    double bow_planform_r = 2.4;               // planform (b/hb)^q + t^r = 1; r > 2 keeps G2 at the shoulder
    double bow_planform_q = 2.0;
    double xi_bow_corner = 0.80;               // bilge corner starts growing forward of this
    double stern_cutup_height = 9.0;           // m, keel rise at AP (propeller aperture region)
    double xi_cutup_end = 0.09;
    double forefoot_rise = 2.5;                // m, keel rise at FP

    double n_aft = 1.55;   // super-ellipse exponent at AP (smaller = more V-shaped)
    double n_fwd = 2.6;    // super-ellipse exponent at FP (larger = fuller U)

    int stations = 41;            // sections used to skin the surface
    int points_per_section = 25;  // sample points per section (keel -> deck edge)
};

struct SectionShape {
    double b;    // half-breadth of the vertical side
    double fb;   // flat-bottom half-width
    double ry;   // bilge corner extent in y
    double rz;   // bilge corner extent in z
    double n;    // super-ellipse exponent
    double zk;   // keel height above baseline
};

SectionShape section_at(const TankerParams& p, double x);
// Points from keel centreline (y = 0) to deck edge (z = D), equally spaced in girth.
std::vector<Vec3> section_points(const SectionShape& s, double x, double D, int npts);
// Station x-positions, clustered toward the ends.
std::vector<double> station_positions(const TankerParams& p);
// Offset table rows: one vector of points per station.
std::vector<std::vector<Vec3>> offsets(const TankerParams& p);
// Skinned bicubic B-spline surface of the port half hull.
BSplineSurface build_hull_surface(const TankerParams& p);

}  // namespace hullcad
