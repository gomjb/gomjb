#pragma once
// Offset tables: hull sections given as points (half-breadth y, height z) per station x.
//
// CSV, one point per row, with a header naming the columns (any order, case-insensitive):
//
//   station,x,point,y,z        <- station and point are optional
//   0,0.000,0,0.000,9.000
//   0,0.000,1,0.512,9.034
//   ...
//
// Separators ',', ';' or tab. Lines starting with '#' are comments; "# units = mm" (or cm, m)
// sets the length unit (default metres). Rows are grouped into stations by the station column,
// or by equal x when there is none. Within a station the points run from the keel (y = 0) to the
// deck edge; a station given deck-first is reversed. Stations are sorted by x and shifted so the
// first one is at x = 0.
#include <string>
#include <vector>

#include "hullcad/cad/hull_param.hpp"
#include "hullcad/core/vec3.hpp"

namespace hullcad {

struct OffsetTable {
    std::vector<std::vector<Vec3>> stations;  // by increasing x, each keel -> deck edge
    double x_shift = 0;                       // added to the file's x values
    std::string source;
};

struct OffsetTableInfo {
    int stations = 0, min_points = 0, max_points = 0;
    double length = 0;      // last station x - first station x
    double max_half_breadth = 0;
    double max_height = 0;
};

// Throws std::invalid_argument with the file name and line number on malformed input.
OffsetTable read_offsets_csv(const std::string& path);
void write_offsets_csv(const OffsetTable& t, const std::string& path);
OffsetTableInfo describe(const OffsetTable& t);

// Main dimensions taken from the table (Lpp = station range, B = 2 x max half-breadth,
// D = deck-edge height of the widest station); the rest of `base` (draft, bilge radius used
// for the block seams, ...) is kept.
TankerParams params_for_table(const OffsetTable& t, TankerParams base);

// Offset table of the parametric hull: `points` per station, optional uniform noise of
// +-noise metres on the interior points (to imitate measured / hand-entered offsets) and
// rounding to `round_to` metres (0 = no rounding).
OffsetTable parametric_offset_table(const TankerParams& p, int points, double noise = 0, double round_to = 0,
                                    unsigned seed = 1);

}  // namespace hullcad
