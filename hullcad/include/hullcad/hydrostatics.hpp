#pragma once
// Hydrostatics and intact stability solvers, written against ComputeBackend batches.
#include <vector>

#include "hullcad/compute_backend.hpp"

namespace hullcad {

struct ShipRef {
    double Lpp = 0, B = 0;
    double x_ref = 0;          // reference x for draft (usually midship, Lpp/2)
    double rho = 1.025;        // t/m^3, sea water
};

// Waterplane with draft `draft` at (x_ref, y = 0); heel positive = starboard down,
// trim positive = bow down (by the head). Angles in degrees.
WaterPlane make_plane(double draft, double heel_deg, double trim_deg, double x_ref);
// Same orientation, shifted so that dot(n, p) = h.
WaterPlane plane_with_offset(const WaterPlane& base, double h);

struct HydroResult {
    double draft = 0, heel = 0, trim = 0;
    double volume = 0, displacement = 0;          // m^3, t
    Vec3 B;                                        // centre of buoyancy (ship frame)
    double Awp = 0;                                // waterplane area
    Vec3 F;                                        // centre of flotation (ship frame)
    double I_T = 0, I_L = 0;                       // waterplane inertia about F
    double BM_T = 0, BM_L = 0, KM_T = 0, KM_L = 0;
    double TPC = 0, MCT = 0;                       // t/cm, t*m/cm
    double Cb = 0, Cw = 0;
};

HydroResult derive(const HydroAccum& a, const WaterPlane& wp, const ShipRef& ref,
                   double draft, double heel, double trim);

// Upright hydrostatic table at even keel, one row per draft (evaluated as one batch).
std::vector<HydroResult> hydrostatic_table(ComputeBackend& be, const ShipRef& ref,
                                           const std::vector<double>& drafts);

struct GZPoint {
    double heel = 0;
    double GZ = 0;       // m, righting lever
    double draft = 0;    // equilibrium draft at x_ref on the centreline
    int iterations = 0;
};

// Fixed-trim GZ curve: for every heel angle find the waterplane that gives `volume`
// (Newton iteration on h, dV/dh = Awp), all heel angles advanced together as one batch.
std::vector<GZPoint> gz_curve(ComputeBackend& be, const ShipRef& ref, double volume, Vec3 G,
                              const std::vector<double>& heels_deg, double trim_deg = 0.0,
                              double tol_rel = 1e-9, int max_iter = 50);

}  // namespace hullcad
