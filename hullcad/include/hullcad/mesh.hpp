#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hullcad/bspline.hpp"

namespace hullcad {

struct TriMesh {
    std::vector<Vec3> verts;
    std::vector<std::array<uint32_t, 3>> tris;  // counter-clockwise seen from outside
};

// Closed, outward-oriented mesh of the full hull: port surface, mirrored starboard surface,
// flat deck at z = D and transverse end caps at AP and FP.
TriMesh tessellate_hull(const BSplineSurface& half_hull, int nu, int nv);

// Box barge L x B x D with its keel on z = 0, centred on y = 0, from x = 0 to x = L.
TriMesh make_box(double L, double B, double D, int nx = 4, int ny = 4, int nz = 4);

// True if every directed edge has exactly one opposite partner (closed, consistently oriented).
bool is_closed_and_consistent(const TriMesh& m);
double signed_volume(const TriMesh& m);

void write_obj(const TriMesh& m, const std::string& path);

}  // namespace hullcad
