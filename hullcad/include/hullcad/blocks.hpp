#pragma once
// Block division: zones (seams), steel weight/geometry model, yard-driven feasibility checks,
// automatic butt selection and rule reports.
//
// A "ring" is the hull between two transverse butts. Longitudinal seams split each ring into
// zone blocks: in the cargo region double bottom, sides, bulkheads and deck (P/S); in the aft
// and fore bodies lower and upper blocks (P/S). Every block is then routed through the yard's
// production stages (yard.hpp) and must fit each enabled stage.
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "hullcad/parallel.hpp"
#include "hullcad/structure.hpp"
#include "hullcad/yard.hpp"

namespace hullcad {

// Section rectangle (y, z) a block occupies inside a ring. The zones of one scheme partition
// the section. plane = the block's main skin: 'h' horizontal (bottom, deck), 'v' vertical.
struct Zone {
    std::string name;
    double ymin, ymax, zmin, zmax;
    char plane = 'h';
};

// Cargo region: DB-S/P, SIDE-S/P, BHD-S/P, DECK-S/P. The side seam sits inboard of both the
// inner hull and the bilge tangent, so double-bottom blocks stay flat.
std::vector<Zone> default_cargo_zones(const TankerParams& p, const TankerStructure& st, const YardProfile& y);
// Aft and fore bodies: LOW-S/P, UP-S/P split at division.end_split_z.
std::vector<Zone> default_end_zones(const TankerParams& p, const YardProfile& y);

struct Mass {
    double w = 0, mx = 0, my = 0, mz = 0;  // t, t*m
    void add(double dw, double x, double y, double z) { w += dw; mx += dw * x; my += dw * y; mz += dw * z; }
    Mass& operator+=(const Mass& o) { w += o.w; mx += o.mx; my += o.my; mz += o.mz; return *this; }
    Vec3 cog() const { return w > 0 ? Vec3{mx / w, my / w, mz / w} : Vec3{}; }
};

// Axis-aligned extent in the section plane.
struct Box2 {
    double ymin = std::numeric_limits<double>::infinity(), ymax = -std::numeric_limits<double>::infinity();
    double zmin = std::numeric_limits<double>::infinity(), zmax = -std::numeric_limits<double>::infinity();
    void add(double y, double z) {
        ymin = std::min(ymin, y); ymax = std::max(ymax, y); zmin = std::min(zmin, z); zmax = std::max(zmax, z);
    }
    void merge(const Box2& b) {
        if (b.empty()) return;
        add(b.ymin, b.zmin); add(b.ymax, b.zmax);
    }
    bool empty() const { return ymin > ymax; }
    double width() const { return empty() ? 0 : ymax - ymin; }
    double height() const { return empty() ? 0 : zmax - zmin; }
};

// Steel weight and geometry per zone along the ship. Longitudinal members are integrated over
// thin x slices (prefix sums give any x range in O(1)); transverse members (bulkheads, webs)
// are point items with their own per-zone weight and extent.
class WeightModel {
public:
    struct SliceInfo {
        Box2 extent;       // all members of the zone in this slice
        Box2 flat_h;       // z-range of near-horizontal shell plating (planarity test)
        Box2 flat_v;       // y-range of near-vertical shell plating
        double plate_len = 0;  // girth of plating in the zone (for plate counts)
    };
    struct TransverseItem {
        double x;
        std::string kind;  // "bulkhead" or "web"
        std::vector<Mass> zone;
        std::vector<Box2> extent;
    };

    std::vector<Zone> zones;  // cargo scheme first, then end scheme
    int n_cargo_zones = 0;
    double x0 = 0, dx = 0;
    int nslice = 0, nzone = 0;
    std::vector<Mass> prefix;       // (nslice + 1) * nzone cumulative longitudinal mass
    std::vector<SliceInfo> slices;  // nslice * nzone
    std::vector<TransverseItem> transverse;

    // Mass of zone z for x in [xa, xb) (transverse items at xa included, at xb excluded).
    Mass zone_mass(int z, double xa, double xb) const;
    // Whole-ring mass (cargo scheme; both schemes partition the same section).
    Mass ring_mass(double xa, double xb) const;

private:
    Mass prefix_at(int z, double x) const;
};

WeightModel build_weight_model(const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                               const std::vector<Zone>& cargo_zones, const std::vector<Zone>& end_zones,
                               double dx, ThreadPool& pool);

// One yard check on one block: value must not exceed limit.
struct Check {
    std::string name;  // e.g. "transporter.weight"
    double value = 0, limit = 0;
    std::string detail;
    double ratio() const { return limit > 0 ? value / limit : 0.0; }
    bool ok() const { return value <= limit * (1 + 1e-12); }
};

struct Block {
    int ring = 0;
    std::string name, zone;
    double x0 = 0, x1 = 0, fr0 = 0, fr1 = 0;
    Mass mass;                     // net steel
    double transport_weight = 0;   // steel + margin + pre-outfitting
    double lift_weight = 0;        // transport weight + rigging
    double L = 0, B = 0, H = 0;    // envelope, m
    bool flat = false;             // flat block (panel line) vs curved block
    int plate_butts = 0, plate_seams = 0;
    std::vector<Check> checks;
    bool ok() const;
    const Check* binding() const;  // highest utilisation
};

// Evaluates one zone block of [xa, xb) against the yard profile.
Block evaluate_block(const WeightModel& wm, int zone, double xa, double xb, const YardProfile& y);

struct Division {
    std::vector<double> butts;  // x positions including 0 and Lpp
    std::vector<Block> blocks;
    std::vector<std::string> issues;  // rule violations (empty = OK)
};

std::vector<double> butt_candidates(const TankerParams& p, const TankerStructure& st, const YardProfile& y);

// Builds blocks for the given butts and checks every rule.
Division evaluate_division(const WeightModel& wm, const TankerParams& p, const TankerStructure& st,
                           std::vector<double> butts, const YardProfile& y);

// Chooses butts (dynamic programming over candidates): fewest rings such that every block
// passes every enabled yard check, then the smallest sum of squared ring weights (evenly loaded
// rings). Butts just forward of the engine-room bulkhead and just aft of the collision bulkhead
// are mandatory. When no division exists, the issues name the checks that block it.
Division auto_divide(const WeightModel& wm, const TankerParams& p, const TankerStructure& st, const YardProfile& y);

// Per check: how many blocks it binds, and its highest utilisation (bottleneck report).
struct CheckSummary {
    std::string name;
    int binding_blocks = 0;
    double max_ratio = 0;
    std::string worst_block;
};
std::vector<CheckSummary> summarize_checks(const Division& d);

void write_division_csv(const Division& d, const std::string& path);
void write_division_svg(const Division& d, const TriMesh& hull, const TankerParams& p, const TankerStructure& st,
                        const WeightModel& wm, const YardProfile& y, const std::string& path);

}  // namespace hullcad
