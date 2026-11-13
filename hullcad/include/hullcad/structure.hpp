#pragma once
// Reference-based hull structure: members are defined by position (frame number, offset from
// centreline, height above base), and their geometry is generated from the current hull
// surface. Changing the hull form regenerates the structure.
#include <string>
#include <vector>

#include "hullcad/hull_param.hpp"
#include "hullcad/mesh.hpp"

namespace hullcad {

// Frame spacing table. Frame 0 is at x = 0 (AP); each segment's spacing applies from its
// from_frame forward until the next segment.
struct FrameSegment {
    double from_frame;
    double spacing;  // m
};

class FrameTable {
public:
    FrameTable() = default;
    explicit FrameTable(std::vector<FrameSegment> segs);
    double x(double frame) const;
    double frame(double x) const;
    double spacing_at(double frame) const;

private:
    std::vector<FrameSegment> segs_;
    std::vector<double> x0_;  // x at each segment's from_frame
};

struct PlateSpec {
    double t_mm = 0;          // plate thickness
    double stiff_factor = 0;  // stiffener/bracket weight as a fraction of plate weight
};

struct TankerStructure {
    FrameTable frames;
    double double_side = 2.0;   // m, wing tank width (MARPOL I/19: min(0.5 + DWT/20000, 2.0))
    double double_bottom = 2.0; // m, double-bottom height (MARPOL I/19: min(B/15, 2.0))
    double cargo_aft_frame = 50;   // engine-room front bulkhead
    double cargo_fwd_frame = 240;  // collision bulkhead
    std::vector<double> bulkhead_frames;  // all transverse bulkheads
    int web_every = 5;                    // web frame every n frames in the cargo region
    bool centreline_bulkhead = true;
    std::vector<double> db_girder_y;      // double-bottom girders, port offsets (0 = centre girder)
    std::vector<double> side_stringer_z;  // horizontal stringers in the double side
    double web_opening_ratio = 0.30;      // manholes / lightening holes in webs and floors

    // Assumed plate thicknesses for block-weight planning (not CSR scantlings).
    PlateSpec bottom_shell{18.5, 0.30}, side_shell{17.0, 0.30}, deck{18.0, 0.30};
    PlateSpec inner_bottom{19.0, 0.30}, inner_hull{16.5, 0.30}, long_bulkhead{15.0, 0.30};
    PlateSpec db_girder{15.0, 0.10}, stringer{12.0, 0.10};
    PlateSpec trans_bulkhead{15.0, 0.35}, web{14.0, 0.10};

    double cargo_x0() const { return frames.x(cargo_aft_frame); }
    double cargo_x1() const { return frames.x(cargo_fwd_frame); }
    std::vector<double> bulkheads_x() const;
    std::vector<double> webs_x() const;  // cargo-region web frames (excluding bulkhead frames)
};

// 120k DWT tanker layout: aft peak, engine room, slop tanks, 6 pairs of cargo tanks, fore peak.
TankerStructure default_tanker_structure(const TankerParams& p);

// Transverse section of the hull mesh at x (both sides; y to port, z up).
struct Seg2 {
    double y0, z0, y1, z1;
};

struct HullSection {
    std::vector<Seg2> segs;
    double half_breadth(double z) const;  // port half-breadth at height z (0 if none)
    double bottom_z(double y) const;      // lowest hull point above/below offset y
};
HullSection hull_section(const TriMesh& hull, double x);

// Longitudinal member traces in one section, each with its plate spec.
struct MemberTrace {
    Seg2 seg;
    const PlateSpec* spec;
    const char* kind;
};
std::vector<MemberTrace> section_members(const HullSection& s, double x, const TankerParams& p,
                                         const TankerStructure& st);

// Inner-hull half-breadth at height z: 2.0 m inboard of the shell, never wider than B/2 - 2.0.
double inner_hull_y(const HullSection& s, const TankerParams& p, const TankerStructure& st, double z);

}  // namespace hullcad
