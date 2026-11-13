#pragma once
// Per-triangle hydrostatic integrals against an arbitrary (heeled / trimmed) waterplane.
//
// This header is the single source of truth for the math and is compiled both for the CPU
// backend and (when enabled) the CUDA backend, so every backend integrates identically.
//
// Method: for a closed, outward-oriented mesh, the submerged volume is the sum of signed
// tetrahedra (o, p0, pk, pk+1) over the submerged part of each triangle, with the apex o
// placed ON the waterplane. The waterplane cap then contributes zero volume, so it never has
// to be built. The cap's area and inertia come from Green's theorem along the waterline
// segments that the clipping produces.
#include "hullcad/vec3.hpp"

namespace hullcad {

struct WaterPlane {
    Vec3 n;     // unit normal pointing up out of the water (ship-fixed frame)
    double h;   // plane: dot(n, p) = h; submerged where h - dot(n, p) >= 0
    Vec3 o;     // a point on the plane (tetra apex, local origin)
    Vec3 e1;    // in-plane axis, longitudinal (x projected onto the plane)
    Vec3 e2;    // in-plane axis, transverse; e1 x e2 = n
};

struct HydroAccum {
    double V = 0;                 // submerged volume
    double Mx = 0, My = 0, Mz = 0; // first moments of volume about o
    double A = 0;                 // waterplane area
    double Su = 0, St = 0;        // first moments of area (local u = e1, t = e2)
    double Suu = 0, Stt = 0, Sut = 0;  // second moments of area about o
};

HC_HD inline void accum_add(HydroAccum& a, const HydroAccum& b) {
    a.V += b.V; a.Mx += b.Mx; a.My += b.My; a.Mz += b.Mz;
    a.A += b.A; a.Su += b.Su; a.St += b.St;
    a.Suu += b.Suu; a.Stt += b.Stt; a.Sut += b.Sut;
}

HC_HD inline void accumulate_triangle(Vec3 a, Vec3 b, Vec3 c, const WaterPlane& wp, HydroAccum& acc) {
    const Vec3 p[3] = {a - wp.o, b - wp.o, c - wp.o};  // relative to o (on plane)
    double d[3];
    for (int i = 0; i < 3; ++i) d[i] = -dot(wp.n, p[i]);  // = h - n.(p+o) since n.o = h
    if (d[0] < 0 && d[1] < 0 && d[2] < 0) return;

    // Sutherland-Hodgman clip of the triangle to the submerged half-space.
    Vec3 poly[4];
    int np = 0;
    Vec3 exitp, entryp;
    bool cut = false;
    for (int i = 0; i < 3; ++i) {
        int j = (i + 1) % 3;
        bool in_i = d[i] >= 0, in_j = d[j] >= 0;
        if (in_i && in_j) {
            poly[np++] = p[j];
        } else if (in_i && !in_j) {
            double t = d[i] / (d[i] - d[j]);
            exitp = p[i] + (p[j] - p[i]) * t;
            poly[np++] = exitp;
            cut = true;
        } else if (!in_i && in_j) {
            double t = d[i] / (d[i] - d[j]);
            entryp = p[i] + (p[j] - p[i]) * t;
            poly[np++] = entryp;
            poly[np++] = p[j];
        }
    }

    // Volume and its first moment (tetra apex at the local origin o).
    for (int k = 1; k + 1 < np; ++k) {
        double v6 = dot(poly[0], cross(poly[k], poly[k + 1]));
        double v = v6 / 6.0;
        Vec3 cen = (poly[0] + poly[k] + poly[k + 1]) * 0.25;
        acc.V += v;
        acc.Mx += v * cen.x; acc.My += v * cen.y; acc.Mz += v * cen.z;
    }

    // Waterline segment: the cap boundary runs entry -> exit (opposite to the hull boundary).
    if (cut) {
        double u0 = dot(entryp, wp.e1), t0 = dot(entryp, wp.e2);
        double u1 = dot(exitp, wp.e1), t1 = dot(exitp, wp.e2);
        double cr = u0 * t1 - u1 * t0;
        acc.A += 0.5 * cr;
        acc.Su += (u0 + u1) * cr / 6.0;
        acc.St += (t0 + t1) * cr / 6.0;
        acc.Suu += (u0 * u0 + u0 * u1 + u1 * u1) * cr / 12.0;
        acc.Stt += (t0 * t0 + t0 * t1 + t1 * t1) * cr / 12.0;
        acc.Sut += (u0 * t1 + 2.0 * u0 * t0 + 2.0 * u1 * t1 + u1 * t0) * cr / 24.0;
    }
}

}  // namespace hullcad
