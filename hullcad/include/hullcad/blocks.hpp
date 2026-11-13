#pragma once
// Block division: zones (seams), steel weight model, automatic butt selection, rule checks.
//
// A "ring" is the hull between two transverse butts. In the cargo region each ring is split
// by longitudinal seams into zone blocks (double bottom, sides, bulkhead, deck); in the aft
// and fore bodies a ring is one block.
#include <string>
#include <vector>

#include "hullcad/parallel.hpp"
#include "hullcad/structure.hpp"

namespace hullcad {

// Section rectangle (y, z) that a block occupies inside a ring. Zones must partition the section.
struct Zone {
    std::string name;
    double ymin, ymax, zmin, zmax;
};

// Seams are offset from structural members (default 300 mm) so no member lies on a seam.
std::vector<Zone> default_cargo_zones(const TankerParams& p, const TankerStructure& st,
                                      double seam_offset = 0.3, double deck_zone_depth = 3.0);

struct Mass {
    double w = 0, mx = 0, my = 0, mz = 0;  // t, t*m
    void add(double dw, double x, double y, double z) { w += dw; mx += dw * x; my += dw * y; mz += dw * z; }
    Mass& operator+=(const Mass& o) { w += o.w; mx += o.mx; my += o.my; mz += o.mz; return *this; }
    Vec3 cog() const { return w > 0 ? Vec3{mx / w, my / w, mz / w} : Vec3{}; }
};

// Steel weight per zone along the ship. Longitudinal members are integrated over thin x slices
// (prefix sums give any x range in O(1)); transverse members (bulkheads, webs) are point items.
class WeightModel {
public:
    struct TransverseItem {
        double x;
        std::string kind;
        std::vector<Mass> zone;  // per zone
    };

    double x0 = 0, dx = 0;
    int nslice = 0, nzone = 0;
    std::vector<Mass> prefix;  // (nslice + 1) * nzone cumulative longitudinal mass
    std::vector<TransverseItem> transverse;

    // Mass of zone z for x in [xa, xb) (transverse items at xa included, at xb excluded).
    Mass zone_mass(int z, double xa, double xb) const;
    Mass ring_mass(double xa, double xb) const;

private:
    Mass prefix_at(int z, double x) const;
};

WeightModel build_weight_model(const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                               const std::vector<Zone>& zones, double dx, ThreadPool& pool);

struct DivisionRules {
    double max_block_weight = 450.0;  // t, lifting capacity for one block
    double max_ring_length = 25.0;    // m
    double min_ring_length = 6.0;     // m
    double min_butt_clearance = 0.3;  // m, from transverse bulkheads and web frames
};

struct Block {
    int ring = 0;
    std::string name, zone;
    double x0 = 0, x1 = 0, fr0 = 0, fr1 = 0;
    Mass mass;
    bool overweight = false;
};

struct Division {
    std::vector<double> butts;  // x positions including 0 and Lpp
    std::vector<Block> blocks;
    std::vector<std::string> issues;  // rule violations (empty = OK)
};

// Admissible butt positions: midway between frames, clear of transverse members.
std::vector<double> butt_candidates(const TankerParams& p, const TankerStructure& st, const DivisionRules& r);

// Builds blocks for the given butts and checks the rules.
Division evaluate_division(const WeightModel& wm, const std::vector<Zone>& zones, const TankerParams& p,
                           const TankerStructure& st, std::vector<double> butts, const DivisionRules& r);

// Chooses butts (dynamic programming over candidates): fewest rings such that every block meets
// the weight and length rules, then the smallest sum of squared ring weights (evenly loaded
// rings). Butts just forward of the engine-room bulkhead and just aft of the collision
// bulkhead are mandatory.
Division auto_divide(const WeightModel& wm, const std::vector<Zone>& zones, const TankerParams& p,
                     const TankerStructure& st, const DivisionRules& r);

void write_division_csv(const Division& d, const std::string& path);
void write_division_svg(const Division& d, const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                        const std::vector<Zone>& zones, const DivisionRules& r, const std::string& path);

}  // namespace hullcad
