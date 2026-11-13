#pragma once
// Lines plan (body plan, half-breadth plan, sheer plan) drawn by slicing the hull mesh.
#include <string>
#include <utility>
#include <vector>

#include "hullcad/mesh.hpp"

namespace hullcad {

using Segment = std::pair<Vec3, Vec3>;

// Intersection of the mesh with the plane {p : p[axis] = value}; axis 0 = x, 1 = y, 2 = z.
std::vector<Segment> slice(const TriMesh& m, int axis, double value);

struct LinesPlanSpec {
    double Lpp = 0, B = 0, D = 0, T = 0;
    std::vector<double> stations;    // x positions for body plan sections
    std::vector<double> waterlines;  // z positions
    std::vector<double> buttocks;    // y positions (port)
};

void write_lines_plan_svg(const TriMesh& m, const LinesPlanSpec& spec, const std::string& path);

}  // namespace hullcad
