#include "hullcad/lines_plan.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace hullcad {

namespace {
double comp(const Vec3& v, int axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }
}  // namespace

std::vector<Segment> slice(const TriMesh& m, int axis, double value) {
    std::vector<Segment> segs;
    for (auto& t : m.tris) {
        Vec3 p[3] = {m.verts[t[0]], m.verts[t[1]], m.verts[t[2]]};
        double d[3];
        for (int i = 0; i < 3; ++i) d[i] = comp(p[i], axis) - value;
        Vec3 hit[2];
        int nh = 0;
        for (int i = 0; i < 3 && nh < 2; ++i) {
            int j = (i + 1) % 3;
            if ((d[i] < 0) != (d[j] < 0)) {
                double s = d[i] / (d[i] - d[j]);
                hit[nh++] = p[i] + (p[j] - p[i]) * s;
            }
        }
        if (nh == 2) segs.emplace_back(hit[0], hit[1]);
    }
    return segs;
}

void write_lines_plan_svg(const TriMesh& m, const LinesPlanSpec& s, const std::string& path) {
    // Layout (metres, then scaled): sheer plan on top, half-breadth plan below it,
    // body plan on the right (aft sections left of centreline, forward sections right).
    const double gap = 0.06 * s.Lpp;
    const double sheer_y0 = 0.0;                    // top of sheer plan
    const double plan_y0 = s.D + gap;               // top of half-breadth plan
    const double body_x0 = s.Lpp + gap + 0.5 * s.B; // body plan centreline
    const double W = s.Lpp + 2 * gap + s.B, H = s.D + gap + 0.5 * s.B;
    const double k = 1600.0 / W;                    // px per metre
    const double pad = 30.0;

    std::ostringstream o;
    o << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W * k + 2 * pad << "' height='" << H * k + 2 * pad + 20
      << "' viewBox='0 0 " << W * k + 2 * pad << ' ' << H * k + 2 * pad + 20 << "'>\n"
      << "<rect width='100%' height='100%' fill='#fbfaf7'/>\n"
      << "<g font-family='sans-serif' font-size='13' fill='#333'>\n";
    auto line = [&](double x1, double y1, double x2, double y2, const char* col, double w) {
        o << "<line x1='" << pad + x1 * k << "' y1='" << pad + y1 * k << "' x2='" << pad + x2 * k << "' y2='"
          << pad + y2 * k << "' stroke='" << col << "' stroke-width='" << w << "'/>\n";
    };
    const char* grid = "#d5d0c6";
    const char* wl_col = "#1f6fb2";
    const char* st_col = "#b2451f";
    const char* bt_col = "#2e8b57";

    // Reference grids.
    for (double z : s.waterlines) line(0, sheer_y0 + s.D - z, s.Lpp, sheer_y0 + s.D - z, grid, 0.6);
    for (double x : s.stations) {
        line(x, sheer_y0, x, sheer_y0 + s.D, grid, 0.6);
        line(x, plan_y0, x, plan_y0 + 0.5 * s.B, grid, 0.6);
    }
    const double cl = plan_y0 + 0.5 * s.B;  // centreline of the half-breadth plan (at the bottom)
    for (double y : s.buttocks) line(0, cl - y, s.Lpp, cl - y, grid, 0.6);
    line(0, sheer_y0 + s.D - s.T, s.Lpp, sheer_y0 + s.D - s.T, "#7a8fa6", 1.2);  // design waterline

    // Sheer plan: buttocks (y = const) seen from the side, plus centreline profile.
    for (double y : s.buttocks)
        for (auto& g : slice(m, 1, y))
            line(g.first.x, sheer_y0 + s.D - g.first.z, g.second.x, sheer_y0 + s.D - g.second.z, bt_col, 1.0);
    for (auto& g : slice(m, 1, 1e-6))
        line(g.first.x, sheer_y0 + s.D - g.first.z, g.second.x, sheer_y0 + s.D - g.second.z, "#222", 1.4);

    // Half-breadth plan: waterlines (z = const), port side only.
    for (double z : s.waterlines)
        for (auto& g : slice(m, 2, z)) {
            if (g.first.y < 0 || g.second.y < 0) continue;
            line(g.first.x, cl - g.first.y, g.second.x, cl - g.second.y, wl_col, 1.0);
        }

    // Body plan: sections; aft half drawn to the left, forward half to the right.
    for (double z : s.waterlines) line(body_x0 - 0.5 * s.B, s.D - z, body_x0 + 0.5 * s.B, s.D - z, grid, 0.6);
    line(body_x0, 0, body_x0, s.D, "#999", 0.8);
    for (double x : s.stations) {
        bool fwd = x >= 0.5 * s.Lpp;
        for (auto& g : slice(m, 0, x)) {
            if (g.first.y < 0 || g.second.y < 0) continue;
            double sx = fwd ? 1.0 : -1.0;
            line(body_x0 + sx * g.first.y, s.D - g.first.z, body_x0 + sx * g.second.y, s.D - g.second.z, st_col, 1.0);
        }
    }

    o << "<text x='" << pad << "' y='" << H * k + 2 * pad + 8 << "'>Lpp " << s.Lpp << " m, B " << s.B << " m, D "
      << s.D << " m, T " << s.T << " m. Sheer plan (buttocks), half-breadth plan (waterlines), body plan "
      << "(aft | fwd sections).</text>\n</g>\n</svg>\n";

    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << o.str();
}

}  // namespace hullcad
